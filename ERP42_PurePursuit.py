"""ERP42 Racing 실차 제어용 적응형 Pure Pursuit 및 공통 경로 기하 모듈.

입력: 후륜 축 중심의 위치(x, y: m), 차량 방향(yaw: rad, 반시계방향 +),
실측 전진 속도(m/s), 같은 좌표계로 표현된 기준 경로.
출력: 앞바퀴 조향각(rad). 알고리즘 내부에서는 좌회전이 양수(+).
RS232 통신과 하드웨어 입출력은 ERP42_Main.py에서만 처리한다.

제어 이득은 저속 실험을 위한 초기 추정값이며 제조사 인증 보정값이 아니다.
"""
from __future__ import annotations

import csv
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from math import atan2, ceil, cos, floor, hypot, isfinite, pi, radians, sin
from typing import Optional


def clamp(x: float, lo: float, hi: float) -> float:
    return min(max(x, lo), hi)


def wrap_angle(x: float) -> float:
    return (x + pi) % (2.0 * pi) - pi


@dataclass(frozen=True)
class VehicleState:
    x: float  # 후륜 축 중심의 x좌표(m)
    y: float
    yaw: float  # 반시계방향을 양수로 하는 차량 방향(rad)
    speed: float  # 실제 측정한 전진 속도(m/s), 후진 미지원

    def validate(self) -> None:
        if not all(isfinite(v) for v in (self.x, self.y, self.yaw, self.speed)):
            raise ValueError("차량 상태에 유한하지 않은 값(NaN 또는 무한대)이 있습니다")
        if self.speed < -0.03 or self.speed > 20.0 / 3.6 + 0.01:
            raise ValueError("차량 속도가 ERP42 전진 주행 허용 범위를 벗어났습니다")


@dataclass(frozen=True)
class Projection:
    x: float
    y: float
    s: float
    yaw: float
    lateral_error: float  # 차량 기준 왼쪽에 경로가 있으면 양수(횡방향 오차)
    distance: float
    segment_index: int


class ReferencePath:
    """미터 단위의 기준 경로와 사전 계산된 곡률, 구간 최댓값 조회 자료구조.

    경로를 불러올 때 한 번 생성한다. 곡률 계산을 20Hz 제어 반복문에서 매번
    수행하지 않아 연산량을 줄인다. 가까운 구간에서 경로가 자기 교차하지 않는다고 가정한다.
    """
    def __init__(self, points: list[tuple[float, float]], *, curvature_spacing: float = 0.25,
                 curvature_half_window: float = 0.5) -> None:
        if len(points) < 3 or curvature_spacing <= 0 or curvature_half_window <= 0:
            raise ValueError("경로 좌표가 최소 3개이고 곡률 샘플 간격은 양수여야 합니다")
        self.points = [(float(x), float(y)) for x, y in points]
        if not all(isfinite(x) and isfinite(y) for x, y in self.points):
            raise ValueError("경로에 올바르지 않은 좌표가 있습니다")
        self.s = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            ds = hypot(b[0] - a[0], b[1] - a[1])
            if not 0.01 <= ds <= 0.80:
                raise ValueError("Waypoint 간격은 0.01~0.80m여야 합니다. 경로를 재샘플링하세요")
            self.s.append(self.s[-1] + ds)
        self.length = self.s[-1]
        if self.length < 2 * curvature_half_window + 0.2:
            raise ValueError("곡률을 추정하기에는 기준 경로가 너무 짧습니다")
        self.curvature_spacing = curvature_spacing
        self.curvature_half_window = curvature_half_window
        count = int(ceil(self.length / curvature_spacing)) + 1
        self._curvature_abs = [abs(self._curvature_3point(min(i * curvature_spacing, self.length)))
                               for i in range(count)]
        self.max_abs_curvature = max(self._curvature_abs)
        # 구간 곡률 최댓값을 위한 희소 테이블: 초기 O(N log N), 조회마다 O(1).
        self._rmq = [self._curvature_abs]
        k = 1
        while (1 << k) <= count:
            prev = self._rmq[-1]
            offset = 1 << (k - 1)
            self._rmq.append([max(prev[j], prev[j + offset])
                              for j in range(len(prev) - offset)])
            k += 1

    @classmethod
    def from_csv(cls, filename: str) -> "ReferencePath":
        with open(filename, newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            if not reader.fieldnames or not {"x", "y"}.issubset(reader.fieldnames):
                raise ValueError("경로 CSV에 x,y 열이 필요합니다(단위 m, 위치 추정과 같은 좌표계)")
            return cls([(float(row["x"]), float(row["y"])) for row in reader])

    def point_at(self, s_query: float) -> tuple[float, float]:
        s_query = clamp(s_query, 0.0, self.length)
        i = min(bisect_right(self.s, s_query) - 1, len(self.s) - 2)
        t = (s_query - self.s[i]) / (self.s[i + 1] - self.s[i])
        a, b = self.points[i], self.points[i + 1]
        return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))

    def _curvature_3point(self, middle_s: float) -> float:
        h = self.curvature_half_window
        mid = clamp(middle_s, h, self.length - h)
        a, b, c = self.point_at(mid - h), self.point_at(mid), self.point_at(mid + h)
        ab, bc, ac = hypot(b[0]-a[0], b[1]-a[1]), hypot(c[0]-b[0], c[1]-b[1]), hypot(c[0]-a[0], c[1]-a[1])
        if min(ab, bc, ac) < 1e-6:
            raise ValueError("곡률 계산에 사용할 세 점이 퇴화했습니다. 경로 좌표를 확인하세요")
        cross = (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
        return 2 * cross / (ab * bc * ac)

    def preview_abs_curvature(self, s_from: float, distance: float) -> float:
        """전방 탐색 구간에서 미리 계산한 곡률 절댓값의 최댓값을 반환한다."""
        lo = int(floor(clamp(s_from, 0, self.length) / self.curvature_spacing))
        hi = int(ceil(clamp(s_from + max(distance, 0), 0, self.length) / self.curvature_spacing))
        lo = min(lo, len(self._curvature_abs)-1)
        hi = min(hi, len(self._curvature_abs)-1)
        width = hi - lo + 1
        k = width.bit_length() - 1
        return max(self._rmq[k][lo], self._rmq[k][hi - (1 << k) + 1])

    def project(self, x: float, y: float, *, hint_s: Optional[float] = None,
                search_radius_m: float = 4.0) -> Projection:
        """좌표를 가장 가까운 경로 선분에 투영하고 경로 진행 위치를 계산한다.

        이전 진행 거리(hint_s)가 있으면 그 주변만 탐색해 연산을 줄이고,
        경로가 겹치는 곳에서 잘못된 지점을 선택할 가능성을 낮춘다.
        최초 호출에서는 전체 경로를 탐색한다.
        """
        if hint_s is None:
            low, high = 0, len(self.points)-2
        else:
            low = max(0, bisect_left(self.s, hint_s-search_radius_m) - 2)
            high = min(len(self.points)-2, bisect_right(self.s, hint_s+search_radius_m))
        best_d2 = float("inf")
        selected = None
        for i in range(low, high+1):
            ax, ay = self.points[i]
            bx, by = self.points[i+1]
            dx, dy = bx-ax, by-ay
            seg_sq = dx*dx+dy*dy
            t = clamp(((x-ax)*dx + (y-ay)*dy)/seg_sq, 0.0, 1.0)
            px, py = ax+t*dx, ay+t*dy
            d2 = (px-x)**2 + (py-y)**2
            if d2 < best_d2:
                seg_len = self.s[i+1] - self.s[i]
                # 기준 경로가 차량의 왼쪽에 있으면 횡방향 오차를 양수로 계산
                signed_error = (dx*(py-y) - dy*(px-x))/seg_len
                best_d2 = d2
                selected = Projection(px, py, self.s[i]+t*seg_len, atan2(dy, dx),
                                      signed_error, d2**0.5, i)
        if selected is None:
            raise ValueError("가장 가까운 기준 경로 선분을 찾지 못했습니다")
        return selected


@dataclass(frozen=True)
class PurePursuitConfig:
    wheelbase: float = 1.212
    max_steer_rad: float = radians(20)
    base_ld: float = 1.2         # 전방 주시 기본 거리(m), 제조사 설정값 아님
    speed_gain: float = 0.7      # 속도 반영 계수(s)
    curvature_gain: float = 2.0 # 곡률 보정 계수(m)
    min_ld: float = 1.5
    max_ld: float = 4.0


@dataclass(frozen=True)
class PurePursuitOutput:
    steer_rad: float
    lookahead_m: float
    target_xy: tuple[float, float]
    curvature_abs: float


class PurePursuitController:
    def __init__(self, path: ReferencePath, cfg: PurePursuitConfig | None = None):
        self.path = path
        self.cfg = cfg or PurePursuitConfig()
        if not (0 < self.cfg.min_ld <= self.cfg.max_ld and
                0 < self.cfg.max_steer_rad < pi/2 and self.cfg.wheelbase > 0):
            raise ValueError("Pure Pursuit 설정값이 올바르지 않습니다")

    def control(self, state: VehicleState, rear: Projection, curvature_abs: float) -> PurePursuitOutput:
        ld_speed = clamp(self.cfg.base_ld + self.cfg.speed_gain * state.speed,
                         self.cfg.min_ld, self.cfg.max_ld)
        ld = clamp(ld_speed / (1 + self.cfg.curvature_gain * curvature_abs),
                   self.cfg.min_ld, self.cfg.max_ld)
        tx, ty = self.path.point_at(rear.s + ld)
        dx, dy = tx-state.x, ty-state.y
        distance = hypot(dx, dy)
        if distance < 0.05:
            raise ValueError("Pure Pursuit 목표점이 차량 위치와 너무 가깝습니다")
        alpha = wrap_angle(atan2(dy, dx)-state.yaw)
        steer = atan2(2*self.cfg.wheelbase*sin(alpha), distance)
        return PurePursuitOutput(clamp(steer, -self.cfg.max_steer_rad, self.cfg.max_steer_rad),
                                 ld, (tx, ty), curvature_abs)
