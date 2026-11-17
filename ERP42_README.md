# ERP42 Racing: Pure Pursuit + Stanley hybrid real-controller integration

## Three Python files

- `ERP42_Purepursuit.py`: reference path geometry, offline curvature precomputation, velocity/curvature adaptive lookahead, rear-axle-based Pure Pursuit.
- `ERP42_Stanley.py`: front-axle-based Stanley lateral steering controller.
- `ERP42_Main.py`: curvature-based switching, hysteresis, switch confirmation, steering crossfade / slew limit, serial PCU packet I/O, UDP pose input, safety checks and braking attempt.

The files implement a **hardware I/O path**, but this is **not an OEM-approved or safety-certified autonomous vehicle package**. It has not been validated against an actual ERP42. Never drive it before on-site calibration and supervised low-speed testing. Use a functioning physical emergency-stop switch and a clear restricted test area. The supplied platform manual has conflicting steering conversion scales: chassis steering ratio 1/100 vs protocol steering = physical angle *71. Do not guess. The motor raw speed command 0..1000 is not a speed in m/s; no verified mapping was provided.

## Confirmed by supplied ERP42 Racing manual

- Rear-wheel drive, front-wheel steering; wheelbase `1.212 m`; front wheel ±`20 deg`.
- Software-limited speed `20 km/h`. This **is not** the initial driving-speed target.
- PCU RS232, 115200 8N1, 50ms frames: HLV->PCU `14 bytes` **big-endian**, PCU->HLV `20 bytes` **little-endian**.
- `steer` raw range `-2000..2000`, negative = left.
- HLV `SPEED` field `0..1000` is **motor raw**, PCU reports wheel RPM. Rear wheel diameter nominal `0.58m` and rolling diameter requires field verification.
- `BRAKE` 1 = unbraked, 100 = full brake; if alive count does not update for ~0.3sec, Auto Fail full braking is expected from the PCU. Validate firmware behavior in a supervised test.
- Control packet fields AorM, ESTOP, GEAR, SPEED, STEER, BRAKE, ALIVE. HLV AorM must transition 0 to 1 with remote switch in Auto mode.

## Install and test (no hardware connected)

```bash
python ERP42_Main.py --self-test
python ERP42_Main.py --make-example-path example_path.csv
```

The self-test covers: path geometry, Pure Pursuit and Stanley, exactly two mode switches on straight -> curve -> straight, ±20° limit, 14/20-byte endian framing and wheel-RPM-to-speed conversion. **It does not test actual actuators or brakes**.

### Route CSV (required for live mode)

```csv
x,y
0.0,0.0
0.1,0.0
0.2,0.0
0.3,0.0
```

Route is in metres, same local coordinate system as the position estimate. Waypoints must be strictly ordered in driving direction, spaced 0.01 to 0.80m, ideally smooth and non-self-intersecting. Do not send the automatically created example path to the real platform; the live route must be physically surveyed and checked for obstacles. The program rejects any reference path whose estimated maximum curvature exceeds 1/3.5 m^-1. That value is a *conservative preliminary limit* based on the manual's ambiguous R 3.5m description, not verified cornering capacity.

### Pose input (required for live mode)

Another process (e.g. a ROS localization adapter) must send current vehicle pose to **localhost** UDP port `16742`, at least 20Hz, as UTF-8 JSON:

```json
{"seq": 1539, "stamp_unix_s": 1790000000.123, "x": 4.15, "y": -0.23, "yaw": 0.054}
```

`stamp_unix_s` **must** be the actual UNIX capture timestamp, not the sample above (which is only an example). `seq` must strictly increase. `x,y` must represent the **rear axle center**, yaw is radians CCW-positive, pose must use **same metric frame and origin as route**. Sensor offsets must be applied by the localization adapter. Capture and receive timestamps are checked; stale or invalid input attempts emergency stop. If the pose estimator runs on another computer, synchronize clocks and forward UDP through a local interface. Remote unauthenticated UDP should not be exposed to public networks.

### Measured motor speed map (required for live mode)

Before enabling autonomous driving, measure **under supervision** a feed-forward mapping between requested vehicle speeds in m/s and validated PCU motor raw inputs. CSV:

```csv
speed_mps,raw_command
0.0,0
...MEASURED_MPS..., ...MEASURED_RAW...
...MEASURED_MPS..., ...MEASURED_RAW...
```

The dots are placeholders; the file must contain **actual measured numeric calibration data** in ascending order and at least enough data to cover `0.8 m/s`. There is no safe value to infer from the manual alone. Choose a verified raw ceiling and measure braking and steering response (including signs) separately. The software will refuse to connect without the calibration acknowledgements and raw limits.

### Hardware execution ONLY after physical verification

```bash
pip install pyserial
python ERP42_Main.py \
  --enable-hardware \
  --path surveyed_route.csv \
  --port /dev/ttyUSB0 \
  --speed-map verified_speed_map.csv \
  --max-throttle-raw YOUR_VERIFIED_CEILING \
  --steer-counts-per-deg YOUR_VERIFIED_SCALE \
  --confirm-steering-calibrated \
  --confirm-speed-calibrated \
  --confirm-brake-tested \
  --confirm-pose-frame \
  --confirm-safe-site
```

All uppercase placeholder values must be replaced with experimentally verified numeric values. On Windows the serial port is typically `COMx`. The remote control operator must switch to Auto; the HLV initiates AorM 0->1. Live loop is 20 Hz with alive count increment. At any fault it attempts E-Stop (0->1) and full braking, but **software stop messages cannot replace the hardware E-stop**. Test physical and PCU watchdog behavior first.

## Switching threshold / latency countermeasures

- Initial state: Pure Pursuit on straight path.
- Enter Stanley when look-ahead **maximum absolute route curvature >= 0.075 m^-1** for **2 consecutive 50ms cycles** (~100ms). The physical radius is <=13.3m; initial threshold is a *tuning hypothesis*.
- Return to Pure Pursuit only when curvature **<= 0.045 m^-1** for **8 cycles** (~400ms), and at least 450ms have passed since the previous switch. This keeps it from bouncing between methods near threshold.
- Preview window = clamp(`1.2m + measured_speed*1.0s`, `2.0m`, `5.5m`). The preview detects a curve ahead so the algorithm can switch before entering the bend.
- Path curvature is **precomputed once** at 0.25m arc-length intervals using 3-point geometry; range maximum queries use a sparse table O(1). Nearest path projection is restricted to a window around the last known progress (unless first acquisition). Only the selected lateral algorithm is evaluated each tick.
- When switching, steering interpolates over 0.25s and is rate limited to an initial 45°/s. The motor raw command is separately ramp-limited. This can reduce steering discontinuity, but the parameters need validation against the ERP42 actual steering dynamics; it **does not guarantee absence of delay**.
- Curves use lower target speed initially; hard rejection occurs for stale pose/telemetry, route/heading loss, speed excess, progress jump, or communication scheduling overrun.

### Practical tests before unattended movement

1. Confirm serial framing and E-Stop behavior *with drive wheels safely restrained*, including disconnect, power cut and 0.3sec alive timeout; verify actual platform firmware revision.
2. Confirm front wheel STEER direction and raw scale (71 vs 100 discrepancy), then measured actuator time response and steer-rate bound.
3. Confirm wheel-RPM feedback units, effective wheel rolling circumference, and measured PCU raw command to vehicle speed; calibrate brakes.
4. Check GNSS/SLAM localization rear-axle transform, heading sign/units, timestamps and coordinate frame.
5. On a closed, cleared track, begin supervised low-speed runs and log curvature, switching state, lateral error, steering, speed, mode-change time, and E-stop triggers. Only then tune the adaptive gains / switch thresholds.

## Why this is not turnkey yet

No GNSS/IMU/LiDAR/SLAM sensor data, ROS topics, on-vehicle firmware version, validated steering scale, effective rolling radius, motor throttle calibration, hydraulic brake response or measured latency have been supplied. The software includes actual packet serialization and a physical-vehicle run path, but direct deployment without these calibrations is unsafe and can fail. This implementation is NOT equivalent to final integration testing or validation.
