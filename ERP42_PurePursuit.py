"""ERP42 Racing real-controller core: adaptive Pure Pursuit and shared path geometry.

Inputs: rear-axle-centre pose (metres, radians CCW), measured forward speed m/s,
metric reference path in the SAME frame. Steering output is FRONT-WHEEL angle
in radians, positive LEFT. Serial/hardware I/O lives only in ERP42_Main.py.

The gains are initial low-speed engineering estimates, NOT OEM calibrations.
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
    x: float  # rear axle centre, metres
    y: float
    yaw: float  # CCW positive, radians
    speed: float  # measured forward ground speed, m/s (no reversing)

    def validate(self) -> None:
        if not all(isfinite(v) for v in (self.x, self.y, self.yaw, self.speed)):
            raise ValueError("Non-finite vehicle state")
        if self.speed < -0.03 or self.speed > 20.0 / 3.6 + 0.01:
            raise ValueError("Vehicle speed outside ERP42 forward limits")


@dataclass(frozen=True)
class Projection:
    x: float
    y: float
    s: float
    yaw: float
    lateral_error: float  # path LEFT of vehicle => positive (tangent cross path-vehicle)
    distance: float
    segment_index: int


class ReferencePath:
    """Polyline in local metric coordinates, pre-indexed curvature and RMQ.

    Constructed ONCE when loading a route; expensive curvature calculations are
    not repeated in the 20-Hz control loop. Route is not self-intersecting nearby.
    """
    def __init__(self, points: list[tuple[float, float]], *, curvature_spacing: float = 0.25,
                 curvature_half_window: float = 0.5) -> None:
        if len(points) < 3 or curvature_spacing <= 0 or curvature_half_window <= 0:
            raise ValueError("Require >=3 path points and positive curvature sampling")
        self.points = [(float(x), float(y)) for x, y in points]
        if not all(isfinite(x) and isfinite(y) for x, y in self.points):
            raise ValueError("Path has invalid coordinates")
        self.s = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            ds = hypot(b[0] - a[0], b[1] - a[1])
            if not 0.01 <= ds <= 0.80:
                raise ValueError("Path waypoint spacing must be 0.01..0.80 m; resample beforehand")
            self.s.append(self.s[-1] + ds)
        self.length = self.s[-1]
        if self.length < 2 * curvature_half_window + 0.2:
            raise ValueError("Reference path too short for curvature estimates")
        self.curvature_spacing = curvature_spacing
        self.curvature_half_window = curvature_half_window
        count = int(ceil(self.length / curvature_spacing)) + 1
        self._curvature_abs = [abs(self._curvature_3point(min(i * curvature_spacing, self.length)))
                               for i in range(count)]
        self.max_abs_curvature = max(self._curvature_abs)
        # Sparse table for range maximum. O(N log N) once, O(1) per preview query.
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
                raise ValueError("Path CSV requires columns: x,y (metres, same map frame)")
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
            raise ValueError("Degenerate curvature estimate; check path")
        cross = (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
        return 2 * cross / (ab * bc * ac)

    def preview_abs_curvature(self, s_from: float, distance: float) -> float:
        """Maximum absolute curvature over lookahead, precomputed range max."""
        lo = int(floor(clamp(s_from, 0, self.length) / self.curvature_spacing))
        hi = int(ceil(clamp(s_from + max(distance, 0), 0, self.length) / self.curvature_spacing))
        lo = min(lo, len(self._curvature_abs)-1)
        hi = min(hi, len(self._curvature_abs)-1)
        width = hi - lo + 1
        k = width.bit_length() - 1
        return max(self._rmq[k][lo], self._rmq[k][hi - (1 << k) + 1])

    def project(self, x: float, y: float, *, hint_s: Optional[float] = None,
                search_radius_m: float = 4.0) -> Projection:
        """Project a point onto nearest path segment. Restrict search near prior s.

        For active tracking a hint avoids ambiguities at loop crossings and avoids
        O(number of waypoints) work. First acquisition scans the full route.
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
                # positive when reference path lies LEFT of vehicle
                signed_error = (dx*(py-y) - dy*(px-x))/seg_len
                best_d2 = d2
                selected = Projection(px, py, self.s[i]+t*seg_len, atan2(dy, dx),
                                      signed_error, d2**0.5, i)
        if selected is None:
            raise ValueError("No nearest path projection")
        return selected


@dataclass(frozen=True)
class PurePursuitConfig:
    wheelbase: float = 1.212
    max_steer_rad: float = radians(20)
    base_ld: float = 1.2         # tuning estimates, not manufacturer settings
    speed_gain: float = 0.7      # sec
    curvature_gain: float = 2.0 # m
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
            raise ValueError("Invalid Pure Pursuit configuration")

    def control(self, state: VehicleState, rear: Projection, curvature_abs: float) -> PurePursuitOutput:
        ld_speed = clamp(self.cfg.base_ld + self.cfg.speed_gain * state.speed,
                         self.cfg.min_ld, self.cfg.max_ld)
        ld = clamp(ld_speed / (1 + self.cfg.curvature_gain * curvature_abs),
                   self.cfg.min_ld, self.cfg.max_ld)
        tx, ty = self.path.point_at(rear.s + ld)
        dx, dy = tx-state.x, ty-state.y
        distance = hypot(dx, dy)
        if distance < 0.05:
            raise ValueError("Pure Pursuit target collapsed onto vehicle")
        alpha = wrap_angle(atan2(dy, dx)-state.yaw)
        steer = atan2(2*self.cfg.wheelbase*sin(alpha), distance)
        return PurePursuitOutput(clamp(steer, -self.cfg.max_steer_rad, self.cfg.max_steer_rad),
                                 ld, (tx, ty), curvature_abs)
