"""ERP42 Racing 실차 입출력 및 Pure Pursuit·Stanley 하이브리드 통합 제어.

모듈 구성:
    ERP42_Purepursuit.py : 경로 기하, 곡률 계산, 적응형 Pure Pursuit
    ERP42_Stanley.py     : 전륜 기준 Stanley 횡방향 제어
    ERP42_Main.py        : 알고리즘 전환, 조향 보정, RS232, 실시간 위치 수신, 감시 로직

실행 방식:
    --self-test             : 하드웨어 없이 알고리즘·패킷 자체 검사
    --make-example-path CSV : 예제 경로 생성(실차 주행 경로로 사용 금지)
    --enable-hardware       : 필수 보정 조건 확인 후 실차 RS232 통신 활성화

실시간 위치 정보는 별도의 위치 추정기/ROS 어댑터가 UDP JSON으로 전송한다.
주소: 127.0.0.1:16742, 주기: 최소 20Hz.
예: {"seq":123,"stamp_unix_s":time.time(),"x":1.0,"y":2.0,"yaw":0.1}
x, y: 후륜 축 중심(m). yaw: 반시계방향 양수(rad).
차량 위치와 경로 CSV는 동일한 지도 좌표계를 사용해야 한다.
속도는 모터 Raw 명령이 아니라 PCU 바퀴 RPM 피드백을 변환해서 얻는다.

중요: 실차 연동을 위한 개발 코드이며 안전 인증이나 실차 주행 검증을 마치지 않았다.
매뉴얼(2021-08-19)에는 조향 변환 계수가 71과 100으로 서로 다르게 기재되어 있다.
조향 부호·스케일, 속도 명령 보정표, RPM, 제동, 위치 추정, 비상 정지, 응답 지연을
현장에서 독립 검증하기 전에는 차량을 구동하지 말 것.
정상 동작하는 물리적 비상 정지 장치와 통제된 시험 장소, 감시자가 필요하다.
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
    # 아래 임계값은 제조사 인증값이 아님. 주행·조향 응답 지연 실측 후 튜닝할 것.
    enter_curve: float = 0.075       # 곡률 >= 0.075(1/m): Stanley 진입(R 약 13.3m 이하)
    exit_curve: float = 0.045        # 곡률 <= 0.045(1/m): Pure Pursuit 복귀(R 약 22.2m 이상)
    enter_frames: int = 2           # 20Hz 기준 연속 2회(약 100ms)
    exit_frames: int = 8            # 20Hz 기준 연속 8회(약 400ms)
    min_stanley_hold_s: float = 0.45
    transition_blend_s: float = 0.25
    max_steering_rate_rad_s: float = math.radians(45.0)  # 초기 조향 변화율 상한(실측 필요)
    preview_base_m: float = 1.2
    preview_time_s: float = 1.0      # 속도 × 1초 + 기본 거리만큼 전방 탐색
    min_preview_m: float = 2.0
    max_preview_m: float = 5.5
    max_path_curvature: float = 1.0/3.5  # 경로 검사용 잠정 한계, 회전 반경 정의 확인 필요
    max_pose_age_s: float = 0.20
    max_feedback_age_s: float = 0.20
    max_lateral_offset_m: float = 1.0
    max_heading_error_rad: float = math.radians(40.0)
    max_progress_jump_m: float = 0.65    # 50ms당 진행 거리 급증 감지
    max_progress_backtrack_m: float = 0.5
    straight_speed_mps: float = 0.8     # 직선 초기 목표 속도(안전 속도 인증값 아님)
    curve_speed_mps: float = 0.5
    max_auto_speed_mps: float = 1.2
    max_lateral_accel_mps2: float = 0.35
    hard_speed_margin_mps: float = 0.25
    control_period_s: float = 0.05
    wheel_diameter_m: float = 0.58     # 매뉴얼 후륜 타이어 외경, 실제 구름지름 실측 필요
    overspeed_brake_deadband_mps: float = 0.15
    speed_p_raw_per_mps: float = 80.0  # 속도 피드백 보정 계수, 실측 후 조정
    max_throttle_ramp_raw_s: float = 60.0  # 가속 Raw 명령의 초당 증가량 제한
    max_pose_capture_age_s: float = 0.20


class ModeSwitcher:
    """곡률 히스테리시스·연속 판정·최소 유지 시간을 적용한다.

    전환 과정에서 대기(sleep)나 객체 재생성을 수행하지 않는다.
    조향 명령의 연속성은 메인 제어 반복문 내 SteeringShaper가 담당한다.
    """
    def __init__(self, cfg: HybridConfig):
        if not (0 < cfg.exit_curve < cfg.enter_curve < cfg.max_path_curvature):
            raise ValueError("곡률 전환 임계값은 0 < 복귀 < 진입 < 경로 최대 곡률을 만족해야 합니다")
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
    """모드 전환 시 조향각 급변을 완화하고 초당 조향 명령 변화량을 제한한다."""
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
    """HLV→PCU 송신 패킷: 14바이트, 빅엔디언(ERP42 매뉴얼 41쪽)."""
    if auto not in (0, 1) or estop not in (0, 1) or gear not in (0, 1, 2):
        raise ValueError("플랫폼 모드 또는 기어 값이 유효하지 않습니다")
    if not (0 <= speed_raw <= 1000 and -2000 <= steer_raw <= 2000 and
            0 <= brake <= 100 and 0 <= alive <= 255):
        raise ValueError("명령 값이 통신 프로토콜 허용 범위를 벗어났습니다")
    frame = b"STX" + bytes((auto, estop, gear))
    frame += struct.pack(">HhBB", speed_raw, steer_raw, brake, alive)
    frame += b"\r\n"
    assert len(frame) == 14
    return frame


def decode_feedback(frame: bytes, received_at: float) -> Feedback:
    """PCU→HLV 수신 패킷: 20바이트, 리틀엔디언(ERP42 매뉴얼 41쪽)."""
    if len(frame) != 20 or frame[:3] != b"STX" or frame[-2:] != b"\r\n":
        raise ValueError("ERP42 PCU 수신 프레임의 길이·헤더·종료 문자가 올바르지 않습니다")
    aorm, estop, gear = frame[3:6]
    if aorm not in (0, 1) or estop not in (0, 1) or gear not in (0, 1, 2):
        raise ValueError("PCU 수신 프레임의 상태 필드가 올바르지 않습니다")
    wheel_rpm = struct.unpack("<h", frame[6:8])[0]
    steer_raw = struct.unpack("<h", frame[8:10])[0]
    encoder = struct.unpack("<i", frame[11:15])[0]
    battery = struct.unpack("<H", frame[15:17])[0] * 0.1
    if abs(steer_raw) > 2000 or frame[10] > 100:
        raise ValueError("PCU 조향·브레이크 피드백 값이 허용 범위를 벗어났습니다")
    return Feedback(received_at, bool(aorm), bool(estop), gear, wheel_rpm,
                    steer_raw, frame[10], encoder, battery, frame[17])


class ERP42Serial:
    """PCU RS232 통신 어댑터. 실차 연결 시에만 pyserial이 필요하다."""
    def __init__(self, port: str):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial을 설치하세요: pip install pyserial") from exc
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
            raise IOError("RS232 송신 프레임이 끝까지 전송되지 않았습니다")
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
        """최대한 정지를 시도한다. 물리적 비상 정지 장치를 대신할 수는 없다."""
        for _ in range(3):
            try:
                self.send(auto=1, estop=int(emergency), speed_raw=0, steer_raw=0, brake=100)
            except Exception as exc:
                LOG.error("안전 정지 명령 RS232 송신 실패: %s", exc)
            time.sleep(0.05)

    def close(self) -> None:
        self.ser.close()


class ThrottleShaper:
    """가속 명령 증가를 완만하게 제한하고 감속·오류 시 즉시 줄인다."""
    def __init__(self, cfg: HybridConfig):
        self.prev = 0.0
        self.rise_raw_per_s = cfg.max_throttle_ramp_raw_s

    def update(self, requested: int, dt: float) -> int:
        self.prev = min(float(requested), self.prev + self.rise_raw_per_s*clamp(dt, 0.005, 0.10))
        return int(round(self.prev))


class PoseUDP:
    """외부 위치 추정기가 송신하는 로컬 UDP 정보 중 최신 프레임을 사용한다."""
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
                raise ValueError("위치 정보 seq 값은 매번 증가해야 합니다")
            state = (float(obj["x"]), float(obj["y"]), float(obj["yaw"]))
            capture_unix = float(obj["stamp_unix_s"])
            if not all(math.isfinite(v) for v in (*state, capture_unix)):
                raise ValueError("위치 정보 또는 타임스탬프에 유효하지 않은 값이 있습니다")
            # 위치 추정기와 제어기는 동기화된 시스템 시간을 사용해야 한다.
            age = time.time()-capture_unix
            if not -0.05 <= age <= 0.20:
                raise ValueError(f"위치 정보가 오래되었거나 미래 시각입니다: 지연={age:.3f}초")
            self.last_seq = seq
            self.latest = state
            self.latest_time = time.monotonic()
            self.latest_capture_unix_s = capture_unix

    def get(self, now: float, max_age: float) -> tuple[float, float, float]:
        if self.latest is None or now-self.latest_time > max_age:
            raise RuntimeError("위치 정보가 갱신되지 않거나 없습니다. 비상 정지가 필요합니다")
        return self.latest

    def close(self) -> None:
        self.sock.close()


class CalibratedSpeedMap:
    """현장에서 측정한 목표 속도(m/s)→모터 Raw 명령(0~1000) 보정표."""
    def __init__(self, filename: str, max_raw: int):
        with open(filename, newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            if not reader.fieldnames or not {"speed_mps", "raw_command"}.issubset(reader.fieldnames):
                raise ValueError("속도 보정 CSV에 speed_mps,raw_command 열이 필요합니다")
            self.entries = [(float(r["speed_mps"]), int(r["raw_command"])) for r in reader]
        if len(self.entries) < 2 or self.entries[0] != (0.0, 0):
            raise ValueError("속도 보정표는 (0,0)으로 시작하고 0이 아닌 실측값이 있어야 합니다")
        for i, (speed, raw) in enumerate(self.entries):
            if not (math.isfinite(speed) and 0 <= raw <= max_raw) or (i and (speed <= self.entries[i-1][0] or raw <= self.entries[i-1][1])):
                raise ValueError("속도 보정표의 속도와 Raw 값은 검증된 한도 내에서 엄격히 증가해야 합니다")

    def raw_for(self, speed_mps: float) -> int:
        if not 0 <= speed_mps <= self.entries[-1][0]:
            raise ValueError("요청 속도가 실측 보정표의 범위를 벗어났습니다")
        for (v0, r0), (v1, r1) in zip(self.entries, self.entries[1:]):
            if speed_mps <= v1:
                return int(round(r0 + (r1-r0)*(speed_mps-v0)/(v1-v0)))
        return self.entries[-1][1]


def rpm_to_mps(wheel_rpm: int, wheel_diameter_m: float) -> float:
    return wheel_rpm*math.pi*wheel_diameter_m/60.0


def measured_speed_command(target: float, measured: float, mapping: CalibratedSpeedMap,
                           cfg: HybridConfig, max_raw: int) -> tuple[int, int]:
    """속도 피드백 제어 예시. 제어 이득과 제동 응답은 현장에서 검증해야 한다."""
    if measured > cfg.max_auto_speed_mps + cfg.hard_speed_margin_mps:
        raise RuntimeError("차량 속도가 허용 상한을 초과했습니다")
    if measured > target + cfg.overspeed_brake_deadband_mps:
        # 단계적 약제동 예시. 유압 브레이크 반응 확인 필요, 위험 한도 초과 시 비상 정지.
        brake = int(clamp(10 + 50*(measured-target), 10, 40))
        return 0, brake
    raw = mapping.raw_for(target) + int(cfg.speed_p_raw_per_mps*(target-measured))
    return int(clamp(raw, 0, max_raw)), 1  # 통신 규격에서 brake=1은 제동 해제


def target_velocity(mode: str, kappa: float, remaining_m: float, cfg: HybridConfig) -> float:
    base = (cfg.curve_speed_mps if mode == "STANLEY" or kappa >= cfg.enter_curve*0.8
            else cfg.straight_speed_mps)
    curve_limit = math.sqrt(cfg.max_lateral_accel_mps2/max(kappa, 1e-4))
    # 경로 종점에 가까워질수록 목표 속도를 낮춘다.
    end_limit = math.sqrt(2.0*0.30*max(remaining_m-0.5, 0.0))
    return clamp(min(base, curve_limit, end_limit, cfg.max_auto_speed_mps), 0.0, cfg.max_auto_speed_mps)


def check_live_gates(args: argparse.Namespace) -> None:
    if not args.enable_hardware:
        raise RuntimeError("실차에 접근하려면 --enable-hardware 옵션이 필요합니다")
    needed = ("confirm_steering_calibrated", "confirm_speed_calibrated",
              "confirm_brake_tested", "confirm_pose_frame", "confirm_safe_site")
    absent = ["--"+x.replace("_", "-") for x in needed if not getattr(args, x)]
    if absent:
        raise RuntimeError("검증되지 않은 실차 실행 조건: " + ", ".join(absent))
    if args.steer_counts_per_deg is None or not 0 < args.steer_counts_per_deg <= 100:
        raise RuntimeError("현장 검증된 --steer-counts-per-deg 값이 필요합니다(매뉴얼의 71/100 불일치)")
    if args.max_throttle_raw is None or not 1 <= args.max_throttle_raw <= 1000:
        raise RuntimeError("현장 검증된 --max-throttle-raw 값이 필요합니다(제조사 안전 설정값 확인 불가)")
    if not args.path or not args.port or not args.speed_map:
        raise RuntimeError("실차 실행에는 --path, --port, --speed-map 옵션이 필요합니다")


def run_live(args: argparse.Namespace) -> None:
    check_live_gates(args)
    cfg = HybridConfig()
    path = ReferencePath.from_csv(args.path)
    if path.max_abs_curvature > cfg.max_path_curvature:
        raise RuntimeError(f"경로 최대 곡률 {path.max_abs_curvature:.3f}이(가) "
                           f"차량의 잠정 한계 {cfg.max_path_curvature:.3f}(1/m)를 초과합니다")
    speed_map = CalibratedSpeedMap(args.speed_map, args.max_throttle_raw)
    if speed_map.entries[-1][0] < max(cfg.straight_speed_mps, cfg.curve_speed_mps):
        raise RuntimeError("속도 보정표가 기본 주행 목표 속도 범위를 포함하지 않습니다")
    pp = PurePursuitController(path, PurePursuitConfig())
    stanley = StanleyController(path, StanleyConfig())
    switcher = ModeSwitcher(cfg)
    shaper = SteeringShaper(cfg)
    throttle_shaper = ThrottleShaper(cfg)
    pose_in = PoseUDP(args.pose_bind, args.pose_port)
    link = None
    try:
        link = ERP42Serial(args.port)
        LOG.info("ERP42 통신 연결됨. 완전 제동 유지 중. 물리적 비상 정지 사용 가능 여부를 확인하세요.")
        # Auto 진입 조건에 따라 HLV의 AorM=0을 먼저 보내고 AorM=1로 전환한다.
        for _ in range(4):
            link.send(auto=0, brake=100)
            link.poll()
            time.sleep(cfg.control_period_s)
        # Auto 전환 요청 중에도 완전 제동 유지. 조종기 운용자가 Auto로 변경해야 한다.
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
            if now-next_tick > 0.13:  # 제어 주기 2회 이상 지연으로 판단
                raise RuntimeError("제어 실행 시한을 초과했습니다")
            pose_in.poll()
            feedback = link.poll(wait_s=0.012)
            if feedback is None or now-feedback.arrival_time > cfg.max_feedback_age_s:
                raise RuntimeError("최신 PCU 피드백이 없습니다")
            if not feedback.remote_auto or feedback.estop:
                raise RuntimeError("조종기가 AUTO가 아니거나 비상 정지가 활성화되었습니다")
            if not 44.0 <= feedback.battery_v <= 55.0:
                raise RuntimeError("배터리 전압이 예상 범위를 벗어났습니다")
            if feedback.gear not in (0, 1):
                raise RuntimeError("PCU 기어 피드백이 후진 상태를 나타냅니다")
            x, y, yaw = pose_in.get(now, cfg.max_pose_age_s)
            speed = rpm_to_mps(feedback.wheel_rpm, cfg.wheel_diameter_m)
            state = VehicleState(x, y, yaw, speed)
            state.validate()
            if speed > cfg.max_auto_speed_mps + cfg.hard_speed_margin_mps:
                raise RuntimeError("실측 속도가 소프트웨어 속도 상한을 초과했습니다")
            rear = path.project(state.x, state.y, hint_s=last_s)
            if rear.distance > cfg.max_lateral_offset_m or abs(wrap_angle(rear.yaw-state.yaw)) > cfg.max_heading_error_rad:
                raise RuntimeError("차량 위치 또는 진행 방향이 기준 경로와 크게 벗어났습니다")
            if last_s is not None:
                if rear.s-last_s > cfg.max_progress_jump_m or rear.s-last_s < -cfg.max_progress_backtrack_m:
                    raise RuntimeError("위치 추정 또는 경로 진행 거리가 비정상적으로 급변했습니다")
            last_s = rear.s
            remaining = path.length-rear.s
            if remaining < 0.65 and math.hypot(state.x-path.points[-1][0], state.y-path.points[-1][1]) < 0.85:
                LOG.info("경로 종점 도착: 제동 유지")
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
            # 내부 알고리즘의 양수 조향은 좌회전. ERP42 Raw 명령에서는 음수가 좌회전.
            steer_raw = int(round(-math.degrees(steer_rad)*args.steer_counts_per_deg))
            steer_raw = int(clamp(steer_raw, -2000, 2000))
            desired_speed = target_velocity(mode, kappa, remaining, cfg)
            raw_speed, brake = measured_speed_command(desired_speed, speed, speed_map,
                                                      cfg, args.max_throttle_raw)
            raw_speed = throttle_shaper.update(raw_speed, now-last_tick_for_throttle)
            last_tick_for_throttle = now
            link.send(auto=1, gear=0, speed_raw=raw_speed, steer_raw=steer_raw, brake=brake)
            if changed:
                LOG.info("알고리즘 전환 → %s, 경로 진행=%.2fm, 곡률 절댓값=%.3f, 속도=%.2fm/s", mode, rear.s, kappa, speed)
            if now-last_log >= 0.5:
                LOG.info("모드=%s, 진행=%.1f/%.1fm, 곡률=%.3f, 실측 속도=%.2f, 목표 속도=%.2f, 조향=%.1f도, Raw=(%d,%d)",
                         mode, rear.s, path.length, kappa, speed, desired_speed,
                         math.degrees(steer_rad), raw_speed, steer_raw)
                last_log = now
            next_tick += cfg.control_period_s
            remaining_sleep = next_tick-time.monotonic()
            if remaining_sleep > 0:
                time.sleep(remaining_sleep)
    except (KeyboardInterrupt, Exception) as exc:
        LOG.error("실차 제어 오류 발생: %s", exc)
        if link:
            link.safe_stop(emergency=True)
        raise
    finally:
        if link:
            link.close()
        pose_in.close()


def sample_path_points() -> list[tuple[float, float]]:
    """직선 10m → 반경 8m의 좌회전 90도 → 직선 10m. 오프라인 검사용."""
    points = [(i*.1, 0.0) for i in range(101)]
    # 원호 중심 (10, 8), 시작점 (10, 0), 시작 접선 방향 +x.
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
        # 실제 차량 동역학 없이 경로 위 기준 위치로 제어기와 전환 판단을 검사한다.
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
    assert switcher.switches == 2, f"알고리즘 전환이 불필요하게 반복됐습니다: {switcher.switches}회"
    assert abs(path.max_abs_curvature - 1.0/8) < 0.08
    wire = encode_command(auto=1, estop=0, gear=0, speed_raw=123, steer_raw=-400, brake=1, alive=99)
    assert len(wire) == 14 and wire[:3] == b"STX" and wire[-2:] == b"\r\n"
    assert wire[6:8] == b"\x00{" and wire[8:10] == struct.pack(">h", -400)
    frame = b"STX"+bytes([1, 0, 0])+struct.pack("<hhBiHB", 30, -710, 1, 123, 501, 7)+b"\r\n"
    assert len(frame) == 20
    back = decode_feedback(frame, time.monotonic())
    assert back.wheel_rpm == 30 and back.steer_raw == -710 and back.battery_v == 50.1
    assert abs(rpm_to_mps(60, 0.58) - math.pi*.58) < 1e-8
    print(f"통과: 경로·기하·Pure Pursuit·Stanley·전환({switcher.switches}회), "
          f"조향 한계·빅/리틀엔디언 14/20바이트·RPM 변환. 검사 시간축 {last_t:.1f}초")


def create_example_path(filename: str) -> None:
    with open(filename, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        w.writerows(sample_path_points())
    print(f"예제 경로 생성 완료: {filename}. 현장에서 측량·승인한 실차 경로가 아닙니다")


def build_cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ERP42 하이브리드 경로 추종: RS232 실차 통신 + UDP 위치 추정")
    parser.add_argument("--self-test", action="store_true", help="하드웨어에 연결하지 않고 제어기·전환·통신 규격 검사")
    parser.add_argument("--make-example-path", metavar="CSV", help="오프라인 데모용 예제 기준 경로 생성")
    parser.add_argument("--enable-hardware", action="store_true", help="PCU RS232 실차 통신 활성화에 필요")
    parser.add_argument("--path", help="현장에서 검증한 경로 CSV(x,y 열, 단위 m)")
    parser.add_argument("--port", help="ERP42 RS232 포트(예: /dev/ttyUSB0)")
    parser.add_argument("--pose-bind", default="127.0.0.1")
    parser.add_argument("--pose-port", type=int, default=16742)
    parser.add_argument("--speed-map", help="실측 속도 보정표 CSV(speed_mps,raw_command 열)")
    parser.add_argument("--max-throttle-raw", type=int,
                        help="반드시 실측 검증한 저속 가속 Raw 상한(1~1000)")
    parser.add_argument("--steer-counts-per-deg", type=float,
                        help="실측 검증한 조향 변환 계수(count/degree). 매뉴얼 71과 100 불일치")
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
