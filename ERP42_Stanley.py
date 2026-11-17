"""ERP42 Racing real-controller core: Stanley lateral tracking (forward only).

Inputs and signs are identical to ERP42_Purepursuit.py. In Stanley, the
cross-track error is measured at the FRONT axle centre, not rear axle.
Hardware I/O and safety interlocks are implemented in ERP42_Main.py.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import atan, cos, pi, radians, sin

from ERP42_Purepursuit import ReferencePath, VehicleState, Projection, clamp, wrap_angle


@dataclass(frozen=True)
class StanleyConfig:
    wheelbase: float = 1.212
    max_steer_rad: float = radians(20)
    cross_track_gain: float = 1.3  # 1/s, initial tuning value, not OEM
    softening_speed: float = 0.5  # m/s, initial low-speed stabilization


@dataclass(frozen=True)
class StanleyOutput:
    steer_rad: float
    cross_track_error_m: float
    heading_error_rad: float
    nearest_s: float
    saturated: bool


class StanleyController:
    def __init__(self, path: ReferencePath, cfg: StanleyConfig | None = None):
        self.path = path
        self.cfg = cfg or StanleyConfig()
        if not (self.cfg.wheelbase > 0 and 0 < self.cfg.max_steer_rad < pi/2 and
                self.cfg.cross_track_gain > 0 and self.cfg.softening_speed > 0):
            raise ValueError("Invalid Stanley configuration")

    def control(self, state: VehicleState, rear: Projection) -> StanleyOutput:
        front_x = state.x + self.cfg.wheelbase * cos(state.yaw)
        front_y = state.y + self.cfg.wheelbase * sin(state.yaw)
        projected = self.path.project(front_x, front_y, hint_s=rear.s)
        heading_error = wrap_angle(projected.yaw - state.yaw)
        # e>0 means route is to vehicle's left: positive delta turns left.
        correction = atan(self.cfg.cross_track_gain * projected.lateral_error /
                          (max(0.0, state.speed) + self.cfg.softening_speed))
        raw_delta = heading_error + correction
        return StanleyOutput(
            steer_rad=clamp(raw_delta, -self.cfg.max_steer_rad, self.cfg.max_steer_rad),
            cross_track_error_m=projected.lateral_error,
            heading_error_rad=heading_error,
            nearest_s=projected.s,
            saturated=abs(raw_delta) > self.cfg.max_steer_rad,
        )
