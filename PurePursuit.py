        """전방 여러 지점 중 가장 큰 절댓값 곡률을 사용 (커브 진입 전 대응)."""
        sample_count = 8
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

    def control(self, state: VehicleState) -> tuple[float, float, float, tuple[float, float]]:
        """(조향각 rad, Ld m, 기준 경로 곡률 1/m, 목표점 좌표) 반환."""
        s_now = self.path.nearest_s(state.x, state.y)

        # ① 속도 비례형: 속도 증가 -> 전방 주시 거리 증가
        ld_speed = self.cfg.base_ld + self.cfg.speed_gain * abs(state.speed)

        # ② 거리 제한: 속도 기반 Ld가 너무 작거나 커지지 않게 제한
        ld_speed = clamp(ld_speed, self.cfg.min_ld, self.cfg.max_ld)

        # ③ 곡률 기반: 전방 곡률 증가 -> 전방 주시 거리 감소
        kappa = self.path.upcoming_curvature(s_now, ld_speed, self.cfg.curvature_step)
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
                                                  points[1][0] - points[0][0]), speed=2.0)
    dt = 0.05
    history = []
    for step in range(2500):
        steer, ld, kappa, target = controller.control(state)
        history.append((state.x, state.y, ld, kappa, state.speed))

        # 아래의 속도 조절은 시뮬레이션 시연용 별도 종방향 제어 예시임.
        # Pure Pursuit 자체는 감속을 수행하지 않고 전달된 speed만 사용함.
        desired_speed = clamp(3.0 / (1.0 + 10.0 * abs(kappa)), 1.0, 3.0)
        acceleration = clamp((desired_speed - state.speed) * 2.0, -1.0, 1.0)
        state.speed = max(0.0, state.speed + acceleration * dt)

        # 후륜 중심 기반 운동학 자전거 모델
        state.x += state.speed * cos(state.yaw) * dt
        state.y += state.speed * sin(state.yaw) * dt
        state.yaw = wrap_angle(state.yaw + state.speed / cfg.wheelbase * tan(steer) * dt)

        gx, gy = points[-1]
        if hypot(state.x - gx, state.y - gy) < 0.45:
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
