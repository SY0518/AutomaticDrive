"""ERP42 Racing 기준 속도 비례 + 거리 제한 + 곡률 기반 Pure Pursuit 시뮬레이터.

실행: python ERP42_PurePursuit.py
선택: pip install matplotlib  # 주행 결과 그래프

좌표: m, 속도: m/s, 각도: rad. 차량 기준점은 후륜 축 중심.
참고: 실차 통신/제어 코드가 아닌 시뮬레이터입니다.
      실차에서는 조향 방향/명령 스케일, 상태 추정, 통신 장애, 안전 제어를 별도 검증해야 합니다.
      매뉴얼의 조향비(1/100)와 통신프로토콜 STEER 스케일(71)은 서로 다릅니다.
"""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from math import atan2, cos, hypot, pi, radians, sin, tan


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def wrap_angle(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


@dataclass
class VehicleState:
    x: float  # 후륜 중심 x (m)
    y: float  # 후륜 중심 y (m)
    yaw: float  # 차량 진행 방향 (rad)
    speed: float  # 전진 속도 (m/s)


@dataclass
class PurePursuitConfig:
    # 차량 하드웨어 제원 (ERP42 Racing 매뉴얼 3~4쪽)
    wheelbase: float = 1.212             # 축거 1,212 mm -> m (확정 제원)
    max_steer: float = radians(20.0)     # 바퀴 조향각 제한 ±20 deg (확정 제원)
    vehicle_speed_limit: float = 20 / 3.6  # 제조사 SW 속도 제한 20 km/h -> m/s

    # 아래 값은 '제조사 제원'이 아닌 저속 실험용 초기 가정입니다. 반드시 튜닝하세요.
    base_ld: float = 1.2                # 기본 전방 주시 거리 (m)
    speed_gain: float = 0.7             # 속도 비례 계수 (s)
    curvature_gain: float = 2.0         # 곡률 보정 계수 (m)
    min_ld: float = 1.5                 # 최소 Ld (m)
    max_ld: float = 4.0                 # 최대 Ld (m)
    curvature_step: float = 0.5         # 3점 곡률 계산 간격 (m)
    curvature_samples: int = 8          # 전방 곡률 샘플 수
    control_period: float = 0.05        # 시뮬레이션 제어 주기 (s), 통신 주기 50ms와 일치

    # 주행 가능 여부 검토용 보수적 경로 곡률 기준 (정확한 한계는 실측 필요)
    reference_turning_radius: float = 3.5  # 매뉴얼 표기 R=3.5m


class Path:
    def __init__(self, points: list[tuple[float, float]]):
        if len(points) < 3:
            raise ValueError("경로에는 최소 3개의 좌표가 필요합니다.")
        self.points = [(float(x), float(y)) for x, y in points]
        self.s = [0.0]
        for (x1, y1), (x2, y2) in zip(self.points, self.points[1:]):
            distance = hypot(x2 - x1, y2 - y1)
            if distance < 1e-9:
                raise ValueError("연속된 경로 좌표가 중복되어 있습니다.")
            self.s.append(self.s[-1] + distance)
        self.length = self.s[-1]

    def point_at(self, s_query: float) -> tuple[float, float]:
        """경로의 누적 거리 s 위치를 선형 보간."""
        s_query = clamp(s_query, 0.0, self.length)
        i = min(bisect_right(self.s, s_query) - 1, len(self.s) - 2)
        t = (s_query - self.s[i]) / (self.s[i + 1] - self.s[i])
        x1, y1 = self.points[i]
        x2, y2 = self.points[i + 1]
        return x1 + t * (x2 - x1), y1 + t * (y2 - y1)

    def nearest_s(self, x: float, y: float) -> float:
        """차량 위치를 모든 경로 선분에 투영해 가장 가까운 경로 위치를 탐색."""
        best_dist2 = float("inf")
        best_s = 0.0
        for i, ((ax, ay), (bx, by)) in enumerate(zip(self.points, self.points[1:])):
            dx, dy = bx - ax, by - ay
            t = clamp(((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy), 0.0, 1.0)
            px, py = ax + t * dx, ay + t * dy
            dist2 = (x - px) ** 2 + (y - py) ** 2
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_s = self.s[i] + t * (self.s[i + 1] - self.s[i])
        return best_s

    def curvature_at(self, s_center: float, h: float) -> float:
        """경로의 세 점이 이루는 원의 부호 있는 곡률 κ(1/m)."""
        if self.length < 2.0 * h:
            return 0.0  # 충분한 구간 길이가 없으면 곡률을 안정적으로 계산하지 않음
        # 시작/종료 근처는 3점의 샘플 위치를 안쪽으로 이동 (곡률을 0으로 오인하지 않음)
        s_center = clamp(s_center, h, self.length - h)
        s1, s2, s3 = s_center - h, s_center, s_center + h
        ax, ay = self.point_at(s1)
        bx, by = self.point_at(s2)
        cx, cy = self.point_at(s3)
        ab = hypot(bx - ax, by - ay)
        bc = hypot(cx - bx, cy - by)
        ac = hypot(cx - ax, cy - ay)
        if ab * bc * ac < 1e-9:
            return 0.0
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        return 2.0 * cross / (ab * bc * ac)

    def upcoming_curvature(self, s_now: float, preview: float, h: float, sample_count: int = 8) -> float:
        """전방 여러 지점 중 가장 큰 절댓값 곡률을 사용 (커브 진입 전 대응)."""
        max_kappa = 0.0
        for i in range(sample_count + 1):
            s_sample = s_now + preview * i / sample_count
            kappa = self.curvature_at(s_sample, h)
            if abs(kappa) > abs(max_kappa):
                max_kappa = kappa
        return max_kappa


class AdaptivePurePursuit:
    def __init__(self, path: Path, config: PurePursuitConfig):
        self.path = path
        self.cfg = config
        if not (0 < config.min_ld <= config.max_ld):
            raise ValueError("0 < min_ld <= max_ld 이어야 합니다.")
        if config.wheelbase <= 0 or config.curvature_step <= 0:
            raise ValueError("wheelbase와 curvature_step은 양수여야 합니다.")
        if config.reference_turning_radius <= 0 or config.vehicle_speed_limit <= 0:
            raise ValueError("차량의 회전 반경/속도 제한은 양수여야 합니다.")
        if config.base_ld <= 0 or config.speed_gain < 0 or config.curvature_gain < 0:
            raise ValueError("Ld 계수들을 확인하세요.")
        if config.curvature_samples < 1 or config.control_period <= 0:
            raise ValueError("샘플 개수 및 제어 주기를 확인하세요.")

    def control(self, state: VehicleState) -> tuple[float, float, float, tuple[float, float]]:
        """(조향각 rad, Ld m, 기준 경로 곡률 1/m, 목표점 좌표) 반환."""
        # 소프트웨어 측정/제어 경계: 실차에서는 독립적인 속도 제한 로직도 필요합니다.
        if abs(state.speed) > self.cfg.vehicle_speed_limit + 1e-6:
            raise ValueError("ERP42 Racing 소프트웨어 속도 제한(20 km/h) 초과")
        s_now = self.path.nearest_s(state.x, state.y)

        # ① 속도 비례형: 속도 증가 -> 전방 주시 거리 증가
        ld_speed = self.cfg.base_ld + self.cfg.speed_gain * abs(state.speed)

        # ② 거리 제한: 속도 기반 Ld가 너무 작거나 커지지 않게 제한
        ld_speed = clamp(ld_speed, self.cfg.min_ld, self.cfg.max_ld)

        # ③ 곡률 기반: 전방 곡률 증가 -> 전방 주시 거리 감소
        kappa = self.path.upcoming_curvature(
            s_now, ld_speed, self.cfg.curvature_step, self.cfg.curvature_samples
        )
        ld = ld_speed / (1.0 + self.cfg.curvature_gain * abs(kappa))

        # ④ 곡률 조정 후에도 최소/최대 거리 제한 적용
        ld = clamp(ld, self.cfg.min_ld, self.cfg.max_ld)

        # ⑤ 경로를 따라 Ld만큼 앞선 지점을 목표점으로 선택
        tx, ty = self.path.point_at(s_now + ld)
        dx, dy = tx - state.x, ty - state.y
        alpha = wrap_angle(atan2(dy, dx) - state.yaw)

        # ⑥ Pure Pursuit 조향각: 실제 차량~목표점 직선거리 사용
        #    (경로의 호 길이 Ld와 유클리드 거리는 완전히 같지 않을 수 있음)
        actual_distance = max(hypot(dx, dy), 1e-6)
        steer = atan2(2.0 * self.cfg.wheelbase * sin(alpha), actual_distance)
        steer = clamp(steer, -self.cfg.max_steer, self.cfg.max_steer)
        return steer, ld, kappa, (tx, ty)


def run_demo() -> None:
    # 완만한 S자 곡선이 반복되는 예시 경로
    points = [(i * 0.15, 1.5 * sin(i * 0.15 / 5.0) + 0.5 * sin(i * 0.15 / 1.8))
              for i in range(401)]
    path = Path(points)
    cfg = PurePursuitConfig()
    controller = AdaptivePurePursuit(path, cfg)
    state = VehicleState(x=0.0, y=-0.5, yaw=atan2(points[1][1] - points[0][1],
                                                  points[1][0] - points[0][0]), speed=0.8)
    # 50ms는 매뉴얼의 RS232 프로토콜 주기와 일치합니다. (실차 제어 지연은 별도)
    dt = cfg.control_period
    # 사전 곡률 확인: 물리적으로 좁은 경로는 속도를 낮춰도 따라갈 수 없습니다.
    path_max_curvature = max(
        abs(path.curvature_at(float(s), cfg.curvature_step))
        for s in [path.length * i / 200 for i in range(201)]
    )
    curvature_limit = 1.0 / cfg.reference_turning_radius
    print(f"경로 최대 |곡률|={path_max_curvature:.3f} 1/m, 검토 기준={curvature_limit:.3f} 1/m")
    if path_max_curvature > curvature_limit:
        print("주의: 차량의 명시된 회전 반경 기준보다 급한 구간이 있습니다. 경로 재설계 권장")

    history = []
    for step in range(2500):
        steer, ld, kappa, target = controller.control(state)
        history.append((state.x, state.y, ld, kappa, state.speed))

        # 아래의 속도 조절은 시뮬레이션 시연용 별도 종방향 제어 예시임.
        # Pure Pursuit 자체는 감속을 수행하지 않고 전달된 speed만 사용함.
        desired_speed = clamp(1.5 / (1.0 + 6.0 * abs(kappa)), 0.5, 1.5)
        acceleration = clamp((desired_speed - state.speed) * 2.0, -0.6, 0.6)
        state.speed = clamp(state.speed + acceleration * dt, 0.0, cfg.vehicle_speed_limit)

        # 후륜 중심 기반 운동학 자전거 모델
        state.x += state.speed * cos(state.yaw) * dt
        state.y += state.speed * sin(state.yaw) * dt
        state.yaw = wrap_angle(state.yaw + state.speed / cfg.wheelbase * tan(steer) * dt)

        gx, gy = points[-1]
        if hypot(state.x - gx, state.y - gy) < 0.35:
            print(f"목표 도착: {step * dt:.1f}초, 최종 Ld={ld:.2f}m")
            break
    else:
        print("시뮬레이션 종료: 목표 지점에 도착하지 못함")

    print("첫 5개 기록 (x, y, Ld, 경로곡률, 속도):")
    for record in history[:5]:
        print(tuple(round(value, 3) for value in record))

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("그래프 표시가 필요하면: pip install matplotlib")
        return

    plt.figure(figsize=(10, 5))
    plt.plot([p[0] for p in points], [p[1] for p in points], '--', label='Reference path')
    plt.plot([h[0] for h in history], [h[1] for h in history], label='Vehicle path')
    plt.axis('equal')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Adaptive Pure Pursuit: speed + limits + curvature')
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_demo()
