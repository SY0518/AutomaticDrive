"""ERP42 Racing: real I/O integration + Pure Pursuit / Stanley hybrid control.

Modules:
    ERP42_Purepursuit.py -- path geometry, adaptive Pure Pursuit
    ERP42_Stanley.py      -- Stanley
    ERP42_Main.py         -- switching, command shaping, RS232, live pose, watchdog

Three modes:
    --self-test: offline logic/packet tests; NO hardware connection
    --make-example-path FILE: generates sample route; NEVER use blindly on site
    --enable-hardware: communicate with real ERP42 after explicit calibration gates

Required live pose source (external localization/ROS adapter), UDP JSON to
127.0.0.1:16742, >=20Hz: {"seq":123,"stamp_unix_s":time.time(),"x":1.0,"y":2.0,"yaw":0.1}
 x,y = REAR AXLE CENTER in METRES; yaw = CCW radians, same map frame/path CSV.
Velocity is from verified PCU wheel RPM, not speed command raw.

IMPORTANT: THIS IS INTEGRATION CODE, NOT A VALIDATED VEHICLE SAFETY SYSTEM.
RS232 details match supplied ERP42 Racing manual revision 2021-08-19, but the
manual conflicts about actuator steering scaling (71 vs 100 count/degree).
Do not activate until steering sign/gain, speed raw map, RPM, brakes, localization,
E-stop, command timing, actuator dynamics are independently tested on-site.
An operator with a working physical E-stop and clear test area is required.
"""
from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import socket
import struct
import time
from dataclasses import dataclass
from typing import Optional

from ERP42_Purepursuit import (PurePursuitConfig, PurePursuitController,
                              ReferencePath, VehicleState, clamp, wrap_angle)
from ERP42_Stanley import StanleyConfig, StanleyController

LOG = logging.getLogger("ERP42")


@dataclass(frozen=True)
class HybridConfig:
    # Not OEM values: require track test and steering/latency measurements.
    enter_curve: float = 0.075       # 1/m -> abs R <= 13.3 m: begin Stanley
    exit_curve: float = 0.045        # 1/m -> abs R >= 22.2 m: return PP
    enter_frames: int = 2           # 100 ms at 20Hz
    exit_frames: int = 8            # 400 ms at 20Hz
    min_stanley_hold_s: float = 0.45
    transition_blend_s: float = 0.25
    max_steering_rate_rad_s: float = math.radians(45.0)  # initial conservative setting
    preview_base_m: float = 1.2
    preview_time_s: float = 1.0      # preview at v*1s + base
    min_preview_m: float = 2.0
    max_preview_m: float = 5.5
    max_path_curvature: float = 1.0/3.5  # conservative approx; check physical definition
    max_pose_age_s: float = 0.20
    max_feedback_age_s: float = 0.20
    max_lateral_offset_m: float = 1.0
    max_heading_error_rad: float = math.radians(40.0)
    max_progress_jump_m: float = 0.65    # per 50ms: reject localization discontinuity
    max_progress_backtrack_m: float = 0.5
    straight_speed_mps: float = 0.8     # initial low-speed target, not a safe-speed rating
    curve_speed_mps: float = 0.5
    max_auto_speed_mps: float = 1.2
    max_lateral_accel_mps2: float = 0.35
    hard_speed_margin_mps: float = 0.25
    control_period_s: float = 0.05
    wheel_diameter_m: float = 0.58     # manual rear wheel 200/580R15; verify loaded tire
    overspeed_brake_deadband_mps: float = 0.15
    speed_p_raw_per_mps: float = 80.0  # tune after calibration
    max_throttle_ramp_raw_s: float = 60.0  # smoothly increase motor command
    max_pose_capture_age_s: float = 0.20


class ModeSwitcher:
    """Curvature hysteresis + consecutive samples + dwell.

    No blocking, sleeping or reinitialization in transitions; smooth steering
    interpolation is done by the command shaper in the main loop.
    """
    def __init__(self, cfg: HybridConfig):
        if not (0 < cfg.exit_curve < cfg.enter_curve < cfg.max_path_curvature):
            raise ValueError("Curve thresholds must satisfy 0 < exit < enter < max path curvature")
        self.cfg = cfg
        self.mode = "PURE_PURSUIT"
        self.enter_counter = 0
        self.exit_counter = 0
        self.last_switch_at = float("-inf")
        self.switches = 0

    def update(self, abs_kappa: float, now: float) -> tuple[str, bool]:
        changed = False
        if self.mode == "PURE_PURSUIT":
            self.enter_counter = self.enter_counter + 1 if abs_kappa >= self.cfg.enter_curve else 0
            if self.enter_counter >= self.cfg.enter_frames:
                self.mode = "STANLEY"
                self.enter_counter = 0
                self.exit_counter = 0
                self.last_switch_at = now
                self.switches += 1
                changed = True
        else:
            self.exit_counter = self.exit_counter + 1 if abs_kappa <= self.cfg.exit_curve else 0
            if (self.exit_counter >= self.cfg.exit_frames and
                    now - self.last_switch_at >= self.cfg.min_stanley_hold_s):
                self.mode = "PURE_PURSUIT"
                self.enter_counter = 0
                self.exit_counter = 0
                self.last_switch_at = now
                self.switches += 1
                changed = True
        return self.mode, changed


class SteeringShaper:
    """Avoid jumps on mode switches and limit commanded steering change rate."""
    def __init__(self, cfg: HybridConfig):
        self.cfg = cfg
        self.prev = 0.0
        self.switch_at = float("-inf")
        self.switch_from = 0.0

    def update(self, target: float, now: float, changed: bool, dt: float) -> float:
        if changed:
            self.switch_at = now
            self.switch_from = self.prev
        progress = clamp((now-self.switch_at)/self.cfg.transition_blend_s, 0.0, 1.0)
        target = self.switch_from*(1.0-progress) + target*progress
        max_step = self.cfg.max_steering_rate_rad_s * clamp(dt, 0.005, 0.10)
        self.prev = clamp(target, self.prev-max_step, self.prev+max_step)
        return clamp(self.prev, -math.radians(20), math.radians(20))


@dataclass(frozen=True)
class Feedback:
    arrival_time: float
    remote_auto: bool
    estop: bool
    gear: int
    wheel_rpm: int
    steer_raw: int
    brake: int
    encoder: int
    battery_v: float
    alive: int


def encode_command(*, auto: int, estop: int, gear: int, speed_raw: int,
                   steer_raw: int, brake: int, alive: int) -> bytes:
    """HLV -> PCU: 14 bytes; big endian per supplied ERP42 manual page 41."""
    if auto not in (0, 1) or estop not in (0, 1) or gear not in (0, 1, 2):
        raise ValueError("Invalid platform mode/gear")
    if not (0 <= speed_raw <= 1000 and -2000 <= steer_raw <= 2000 and
            0 <= brake <= 100 and 0 <= alive <= 255):
        raise ValueError("Command field outside wire protocol limits")
    frame = b"STX" + bytes((auto, estop, gear))
    frame += struct.pack(">HhBB", speed_raw, steer_raw, brake, alive)
    frame += b"\r\n"
    assert len(frame) == 14
    return frame


def decode_feedback(frame: bytes, received_at: float) -> Feedback:
    """PCU -> HLV: 20 bytes; little endian per supplied ERP42 manual page 41."""
    if len(frame) != 20 or frame[:3] != b"STX" or frame[-2:] != b"\r\n":
        raise ValueError("Invalid ERP42 PCU frame")
    aorm, estop, gear = frame[3:6]
    if aorm not in (0, 1) or estop not in (0, 1) or gear not in (0, 1, 2):
        raise ValueError("Invalid PCU status fields")
    wheel_rpm = struct.unpack("<h", frame[6:8])[0]
    steer_raw = struct.unpack("<h", frame[8:10])[0]
    encoder = struct.unpack("<i", frame[11:15])[0]
    battery = struct.unpack("<H", frame[15:17])[0] * 0.1
    if abs(steer_raw) > 2000 or frame[10] > 100:
        raise ValueError("Out-of-range PCU steering/brake feedback")
    return Feedback(received_at, bool(aorm), bool(estop), gear, wheel_rpm,
                    steer_raw, frame[10], encoder, battery, frame[17])


class ERP42Serial:
    """PCU serial wire adapter; pyserial is required only for --enable-hardware."""
    def __init__(self, port: str):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("Install pyserial: pip install pyserial") from exc
        self.ser = serial.Serial(port=port, baudrate=115200, bytesize=8,
                                 parity=serial.PARITY_NONE, stopbits=1,
                                 timeout=0.002, write_timeout=0.015)
        self.rx = bytearray()
        self.alive = 0
        self.last_feedback: Optional[Feedback] = None

    def send(self, *, auto: int, estop: int = 0, gear: int = 0,
             speed_raw: int = 0, steer_raw: int = 0, brake: int = 1) -> None:
        frame = encode_command(auto=auto, estop=estop, gear=gear,
                               speed_raw=speed_raw, steer_raw=steer_raw,
                               brake=brake, alive=self.alive)
        count = self.ser.write(frame)
        if count != len(frame):
            raise IOError("Short RS232 frame write")
        self.alive = (self.alive+1) & 0xFF

    def poll(self, wait_s: float = 0.012) -> Optional[Feedback]:
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            raw = self.ser.read(self.ser.in_waiting or 1)
            if raw:
                self.rx.extend(raw)
            while True:
                i = self.rx.find(b"STX")
                if i < 0:
                    del self.rx[:-2]
                    break
                if i:
                    del self.rx[:i]
                if len(self.rx) < 20:
                    break
                candidate = bytes(self.rx[:20])
                if candidate[-2:] != b"\r\n":
                    del self.rx[0]
                    continue
                del self.rx[:20]
                try:
                    self.last_feedback = decode_feedback(candidate, time.monotonic())
                except ValueError:
                    continue
            if self.last_feedback and time.monotonic()-self.last_feedback.arrival_time < 0.01:
                return self.last_feedback
        return self.last_feedback

    def safe_stop(self, *, emergency: bool) -> None:
        """Best effort: do NOT rely on this instead of a hardware physical E-stop."""
        for _ in range(3):
            try:
                self.send(auto=1, estop=int(emergency), speed_raw=0, steer_raw=0, brake=100)
            except Exception as exc:
                LOG.error("Safe-stop serial write failed: %s", exc)
            time.sleep(0.05)

    def close(self) -> None:
        self.ser.close()


class ThrottleShaper:
    """Ramp increases; allow immediate throttle reduction on slowdown/fault."""
    def __init__(self, cfg: HybridConfig):
        self.prev = 0.0
        self.rise_raw_per_s = cfg.max_throttle_ramp_raw_s

    def update(self, requested: int, dt: float) -> int:
        self.prev = min(float(requested), self.prev + self.rise_raw_per_s*clamp(dt, 0.005, 0.10))
        return int(round(self.prev))


class PoseUDP:
    """Local UDP pipe from an external localization estimator; drains to latest frame."""
    def __init__(self, bind: str, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind, port))
        self.sock.setblocking(False)
        self.latest = None
        self.latest_time = float("-inf")
        self.last_seq = -1
        self.latest_capture_unix_s = float("-inf")

    def poll(self) -> None:
        while True:
            try:
                message, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            obj = json.loads(message.decode("utf-8"))
            seq = obj["seq"]
            if not isinstance(seq, int) or isinstance(seq, bool) or seq <= self.last_seq:
                raise ValueError("Localization seq must strictly increase")
            state = (float(obj["x"]), float(obj["y"]), float(obj["yaw"]))
            capture_unix = float(obj["stamp_unix_s"])
            if not all(math.isfinite(v) for v in (*state, capture_unix)):
                raise ValueError("Invalid pose/timestamp")
            # Local estimator and controller should use synchronized system clock.
            age = time.time()-capture_unix
            if not -0.05 <= age <= 0.20:
                raise ValueError(f"Old/future-dated pose sample: age={age:.3f}s")
            self.last_seq = seq
            self.latest = state
            self.latest_time = time.monotonic()
            self.latest_capture_unix_s = capture_unix

    def get(self, now: float, max_age: float) -> tuple[float, float, float]:
        if self.latest is None or now-self.latest_time > max_age:
            raise RuntimeError("Localization stale or absent: emergency stop")
        return self.latest

    def close(self) -> None:
        self.sock.close()


class CalibratedSpeedMap:
    """Site-verified feed-forward map speed[m/s] -> PCU motor raw[0..1000]."""
    def __init__(self, filename: str, max_raw: int):
        with open(filename, newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            if not reader.fieldnames or not {"speed_mps", "raw_command"}.issubset(reader.fieldnames):
                raise ValueError("Speed map CSV needs speed_mps,raw_command")
            self.entries = [(float(r["speed_mps"]), int(r["raw_command"])) for r in reader]
        if len(self.entries) < 2 or self.entries[0] != (0.0, 0):
            raise ValueError("Speed map must start at 0,0 and contain measured nonzero point(s)")
        for i, (speed, raw) in enumerate(self.entries):
            if not (math.isfinite(speed) and 0 <= raw <= max_raw) or (i and (speed <= self.entries[i-1][0] or raw <= self.entries[i-1][1])):
                raise ValueError("Speed calibration map must increase strictly within validated raw limit")

    def raw_for(self, speed_mps: float) -> int:
        if not 0 <= speed_mps <= self.entries[-1][0]:
            raise ValueError("Requested speed exceeds calibrated speed map")
        for (v0, r0), (v1, r1) in zip(self.entries, self.entries[1:]):
            if speed_mps <= v1:
                return int(round(r0 + (r1-r0)*(speed_mps-v0)/(v1-v0)))
        return self.entries[-1][1]


def rpm_to_mps(wheel_rpm: int, wheel_diameter_m: float) -> float:
    return wheel_rpm*math.pi*wheel_diameter_m/60.0


def measured_speed_command(target: float, measured: float, mapping: CalibratedSpeedMap,
                           cfg: HybridConfig, max_raw: int) -> tuple[int, int]:
    """Example bounded speed regulation; gains/brake response MUST be validated on-site."""
    if measured > cfg.max_auto_speed_mps + cfg.hard_speed_margin_mps:
        raise RuntimeError("Excessive vehicle speed")
    if measured > target + cfg.overspeed_brake_deadband_mps:
        # Progressive light brake; site verify hydraulic actuation. Emergency stop on hard limits.
        brake = int(clamp(10 + 50*(measured-target), 10, 40))
        return 0, brake
    raw = mapping.raw_for(target) + int(cfg.speed_p_raw_per_mps*(target-measured))
    return int(clamp(raw, 0, max_raw)), 1  # brake 1 = released per protocol


def target_velocity(mode: str, kappa: float, remaining_m: float, cfg: HybridConfig) -> float:
    base = (cfg.curve_speed_mps if mode == "STANLEY" or kappa >= cfg.enter_curve*0.8
            else cfg.straight_speed_mps)
    curve_limit = math.sqrt(cfg.max_lateral_accel_mps2/max(kappa, 1e-4))
    # Also reduce target speed when approaching route end.
    end_limit = math.sqrt(2.0*0.30*max(remaining_m-0.5, 0.0))
    return clamp(min(base, curve_limit, end_limit, cfg.max_auto_speed_mps), 0.0, cfg.max_auto_speed_mps)


def check_live_gates(args: argparse.Namespace) -> None:
    if not args.enable_hardware:
        raise RuntimeError("Hardware access requires --enable-hardware")
    needed = ("confirm_steering_calibrated", "confirm_speed_calibrated",
              "confirm_brake_tested", "confirm_pose_frame", "confirm_safe_site")
    absent = ["--"+x.replace("_", "-") for x in needed if not getattr(args, x)]
    if absent:
        raise RuntimeError("Unverified hardware preconditions: " + ", ".join(absent))
    if args.steer_counts_per_deg is None or not 0 < args.steer_counts_per_deg <= 100:
        raise RuntimeError("Specify bench-verified --steer-counts-per-deg (manual conflicts: 71 vs 100)")
    if args.max_throttle_raw is None or not 1 <= args.max_throttle_raw <= 1000:
        raise RuntimeError("Specify bench-verified --max-throttle-raw (no safe OEM value is documented)")
    if not args.path or not args.port or not args.speed_map:
        raise RuntimeError("Live use requires --path, --port and --speed-map")


def run_live(args: argparse.Namespace) -> None:
    check_live_gates(args)
    cfg = HybridConfig()
    path = ReferencePath.from_csv(args.path)
    if path.max_abs_curvature > cfg.max_path_curvature:
        raise RuntimeError(f"Route max curvature {path.max_abs_curvature:.3f} exceeds "
                           f"conservative vehicle bound {cfg.max_path_curvature:.3f} 1/m")
    speed_map = CalibratedSpeedMap(args.speed_map, args.max_throttle_raw)
    if speed_map.entries[-1][0] < max(cfg.straight_speed_mps, cfg.curve_speed_mps):
        raise RuntimeError("Speed calibration map doesn't cover default travel speeds")
    pp = PurePursuitController(path, PurePursuitConfig())
    stanley = StanleyController(path, StanleyConfig())
    switcher = ModeSwitcher(cfg)
    shaper = SteeringShaper(cfg)
    throttle_shaper = ThrottleShaper(cfg)
    pose_in = PoseUDP(args.pose_bind, args.pose_port)
    link = None
    try:
        link = ERP42Serial(args.port)
        LOG.info("ERP42 link open, holding full brake. Verify physical E-STOP is available.")
        # Ensure HLV AorM=0 precedes HLV AorM=1, required by platform Auto handshake.
        for _ in range(4):
            link.send(auto=0, brake=100)
            link.poll()
            time.sleep(cfg.control_period_s)
        # Still full-brake while requesting Auto. Radio operator must switch to Auto.
        for _ in range(4):
            link.send(auto=1, brake=100)
            link.poll()
            time.sleep(cfg.control_period_s)
        next_tick = time.monotonic()
        last_tick = next_tick
        last_tick_for_throttle = next_tick
        last_s = None
        last_log = next_tick
        while True:
            now = time.monotonic()
            if now-next_tick > 0.13:  # missed >2 control periods
                raise RuntimeError("Control scheduling deadline missed")
            pose_in.poll()
            feedback = link.poll(wait_s=0.012)
            if feedback is None or now-feedback.arrival_time > cfg.max_feedback_age_s:
                raise RuntimeError("No fresh PCU feedback")
            if not feedback.remote_auto or feedback.estop:
                raise RuntimeError("Remote is not AUTO, or E-STOP asserted")
            if not 44.0 <= feedback.battery_v <= 55.0:
                raise RuntimeError("Battery voltage outside expected range")
            if feedback.gear not in (0, 1):
                raise RuntimeError("PCU gear reports reverse")
            x, y, yaw = pose_in.get(now, cfg.max_pose_age_s)
            speed = rpm_to_mps(feedback.wheel_rpm, cfg.wheel_diameter_m)
            state = VehicleState(x, y, yaw, speed)
            state.validate()
            if speed > cfg.max_auto_speed_mps + cfg.hard_speed_margin_mps:
                raise RuntimeError("Measured vehicle speed above software safety limit")
            rear = path.project(state.x, state.y, hint_s=last_s)
            if rear.distance > cfg.max_lateral_offset_m or abs(wrap_angle(rear.yaw-state.yaw)) > cfg.max_heading_error_rad:
                raise RuntimeError("Vehicle too far from or misaligned with reference path")
            if last_s is not None:
                if rear.s-last_s > cfg.max_progress_jump_m or rear.s-last_s < -cfg.max_progress_backtrack_m:
                    raise RuntimeError("Localization/path progress jumped unexpectedly")
            last_s = rear.s
            remaining = path.length-rear.s
            if remaining < 0.65 and math.hypot(state.x-path.points[-1][0], state.y-path.points[-1][1]) < 0.85:
                LOG.info("End of path: holding brake")
                link.safe_stop(emergency=False)
                return
            preview = clamp(cfg.preview_base_m + cfg.preview_time_s*speed,
                            cfg.min_preview_m, cfg.max_preview_m)
            kappa = path.preview_abs_curvature(rear.s, preview)
            mode, changed = switcher.update(kappa, now)
            if mode == "STANLEY":
                lateral = stanley.control(state, rear)
                desired_delta = lateral.steer_rad
            else:
                lateral = pp.control(state, rear, kappa)
                desired_delta = lateral.steer_rad
            steer_rad = shaper.update(desired_delta, now, changed, now-last_tick)
            last_tick = now
            # Pure/Stanley internally: +angle = LEFT. ERP42 command: negative = LEFT.
            steer_raw = int(round(-math.degrees(steer_rad)*args.steer_counts_per_deg))
            steer_raw = int(clamp(steer_raw, -2000, 2000))
            desired_speed = target_velocity(mode, kappa, remaining, cfg)
            raw_speed, brake = measured_speed_command(desired_speed, speed, speed_map,
                                                      cfg, args.max_throttle_raw)
            raw_speed = throttle_shaper.update(raw_speed, now-last_tick_for_throttle)
            last_tick_for_throttle = now
            link.send(auto=1, gear=0, speed_raw=raw_speed, steer_raw=steer_raw, brake=brake)
            if changed:
                LOG.info("SWITCH -> %s s=%.2fm abs_kappa=%.3f speed=%.2fm/s", mode, rear.s, kappa, speed)
            if now-last_log >= 0.5:
                LOG.info("mode=%s s=%.1f/%.1f curve=%.3f v=%.2f target=%.2f steer=%.1fdeg raw=(%d,%d)",
                         mode, rear.s, path.length, kappa, speed, desired_speed,
                         math.degrees(steer_rad), raw_speed, steer_raw)
                last_log = now
            next_tick += cfg.control_period_s
            remaining_sleep = next_tick-time.monotonic()
            if remaining_sleep > 0:
                time.sleep(remaining_sleep)
    except (KeyboardInterrupt, Exception) as exc:
        LOG.error("REAL VEHICLE CONTROL FAULT: %s", exc)
        if link:
            link.safe_stop(emergency=True)
        raise
    finally:
        if link:
            link.close()
        pose_in.close()


def sample_path_points() -> list[tuple[float, float]]:
    """10m straight -> 90deg left arc R=8m -> 10m straight; offline unit test only."""
    points = [(i*.1, 0.0) for i in range(101)]
    # Center of arc at (10, 8), circle starts at (10,0), tangent +x.
    for i in range(1, 127):
        theta = -math.pi/2 + (math.pi/2)*i/126
        points.append((10+8*math.cos(theta), 8+8*math.sin(theta)))
    for i in range(1, 101):
        points.append((18.0, 8+i*.1))
    return points


def self_test() -> None:
    path = ReferencePath(sample_path_points())
    cfg = HybridConfig()
    pp, st = PurePursuitController(path), StanleyController(path)
    switcher = ModeSwitcher(cfg)
    shaper = SteeringShaper(cfg)
    modes = []
    last_t = 0.0
    for n in range(600):
        s = (path.length - 2.0)*n/599
        # test controller and mode scheduler with reference pose; no vehicle physics
        px, py = path.point_at(s)
        proj = path.project(px, py, hint_s=s)
        state = VehicleState(px, py, proj.yaw, 0.6)
        state.validate()
        preview = clamp(cfg.preview_base_m+state.speed*cfg.preview_time_s,
                        cfg.min_preview_m, cfg.max_preview_m)
        kappa = path.preview_abs_curvature(proj.s, preview)
        mode, changed = switcher.update(kappa, n*0.05)
        output = st.control(state, proj) if mode == "STANLEY" else pp.control(state, proj, kappa)
        steer = shaper.update(output.steer_rad, n*0.05, changed, 0.05)
        assert math.isfinite(steer) and abs(steer) <= math.radians(20)+1e-9
        modes.append(mode)
        last_t = n*.05
    assert modes[0] == "PURE_PURSUIT" and "STANLEY" in modes and modes[-1] == "PURE_PURSUIT", modes[-10:]
    assert switcher.switches == 2, f"Unexpected mode chatter: {switcher.switches} switches"
    assert abs(path.max_abs_curvature - 1.0/8) < 0.08
    wire = encode_command(auto=1, estop=0, gear=0, speed_raw=123, steer_raw=-400, brake=1, alive=99)
    assert len(wire) == 14 and wire[:3] == b"STX" and wire[-2:] == b"\r\n"
    assert wire[6:8] == b"\x00{" and wire[8:10] == struct.pack(">h", -400)
    frame = b"STX"+bytes([1, 0, 0])+struct.pack("<hhBiHB", 30, -710, 1, 123, 501, 7)+b"\r\n"
    assert len(frame) == 20
    back = decode_feedback(frame, time.monotonic())
    assert back.wheel_rpm == 30 and back.steer_raw == -710 and back.battery_v == 50.1
    assert abs(rpm_to_mps(60, 0.58) - math.pi*.58) < 1e-8
    print(f"PASS: path, geometry, pure pursuit, Stanley, switching ({switcher.switches} transitions), "
          f"steer bound, big/little endian 14/20B, RPM conversion. Simulated timeline {last_t:.1f}s")


def create_example_path(filename: str) -> None:
    with open(filename, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        w.writerows(sample_path_points())
    print(f"Example route written to {filename} -- NOT a surveyed/approved physical route")


def build_cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ERP42 hybrid path tracking: real RS232 + UDP localization")
    parser.add_argument("--self-test", action="store_true", help="test controller / switching / protocol WITHOUT hardware")
    parser.add_argument("--make-example-path", metavar="CSV", help="make OFFLINE demonstration reference path")
    parser.add_argument("--enable-hardware", action="store_true", help="required to open PCU RS232")
    parser.add_argument("--path", help="surveyed route CSV with x,y metre columns")
    parser.add_argument("--port", help="ERP42 RS232 COM port e.g. /dev/ttyUSB0")
    parser.add_argument("--pose-bind", default="127.0.0.1")
    parser.add_argument("--pose-port", type=int, default=16742)
    parser.add_argument("--speed-map", help="site-measured feedforward map speed_mps,raw_command")
    parser.add_argument("--max-throttle-raw", type=int,
                        help="REQUIRED SITE VERIFIED low-speed raw throttle ceiling (1..1000)")
    parser.add_argument("--steer-counts-per-deg", type=float,
                        help="SITE VERIFIED counts/degree (manual has contradictory 71 vs 100)")
    for name in ("steering-calibrated", "speed-calibrated", "brake-tested", "pose-frame", "safe-site"):
        parser.add_argument("--confirm-"+name, action="store_true")
    return parser


def main() -> None:
    args = build_cli().parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    if args.self_test:
        self_test()
    elif args.make_example_path:
        create_example_path(args.make_example_path)
    else:
        run_live(args)


if __name__ == "__main__":
    main()
