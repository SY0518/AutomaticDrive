"""ERP42 Racing 실차 제어용 Stanley 횡방향 경로 추종 알고리즘(전진 전용).

입력 좌표계와 조향 부호는 ERP42_Purepursuit.py와 동일하다.
Stanley의 횡방향 오차는 후륜이 아니라 전륜 축 중심을 기준으로 계산한다.
하드웨어 입출력과 안전 인터록은 ERP42_Main.py에서 담당한다.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import atan, cos, pi, radians, sin

from ERP42_Purepursuit import ReferencePath, VehicleState, Projection, clamp, wrap_angle


@dataclass(frozen=True)
class StanleyConfig:
    wheelbase: float = 1.212
    max_steer_rad: float = radians(20)
    cross_track_gain: float = 1.3  # 횡방향 오차 이득(1/s), 초기 튜닝값
    softening_speed: float = 0.5  # 저속 안정화 상수(m/s), 초기 튜닝값


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
            raise ValueError("Stanley 설정값이 올바르지 않습니다")

    def control(self, state: VehicleState, rear: Projection) -> StanleyOutput:
        front_x = state.x + self.cfg.wheelbase * cos(state.yaw)
        front_y = state.y + self.cfg.wheelbase * sin(state.yaw)
        projected = self.path.project(front_x, front_y, hint_s=rear.s)
        heading_error = wrap_angle(projected.yaw - state.yaw)
        # e>0이면 기준 경로가 차량 왼쪽에 있으므로 양의 조향각(좌회전)을 만든다.
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
