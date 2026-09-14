# Bring-up and tuning

Do these in order. Each step assumes the previous one passed.

## 0. Bench checks (wheels off the ground)

1. Power on with the mushroom switch **pressed** (motor power cut). Green LED
   must light — it is wired to 5 V, not to a GPIO.
2. Serial monitor: `pio device monitor -b 115200`. Any `[INA226] … missing` or
   `[IMU] BNO085 not found` message means an I²C address or wiring problem.
3. `ros2 topic hz /gps_localize/telemetry` → ~10 Hz once the agent connects.
4. Release the mushroom switch. `stop_flags` on the dashboard must clear.
   If `hardware_emergency` shows the opposite of reality, flip
   `E_EMER_ACTIVE_LOW` in `firmware/config/safety.h`.

## 1. Motor direction and wiring

Wheels still off the ground, use the manual buttons on the dashboard:

| Press | Expected |
|---|---|
| ▲ | both wheels turn forward |
| ▼ | both wheels turn backward |
| ▶ | left wheel forward, right wheel backward (turns clockwise / right) |
| ◀ | the opposite |

Wrong direction on one wheel → set `MOTOR_A_INVERT` or `MOTOR_B_INVERT` to
`true` in `config/motor.h` and reflash. Both wheels wrong → swap the motor
polarity or invert both.

If a wheel only buzzes at low power, raise `PWM_MIN_MOVE` (default 180) until
it always starts moving.

## 2. IMU orientation

Watch `ทิศทาง / Heading` on the dashboard and turn the robot by hand:

| Test | Expected |
|---|---|
| Rotate the robot clockwise (seen from above) | heading number **increases** (0 → 90 → 180) |
| Point the robot at a landmark and drive forward 10 m | heading settles near the real compass bearing after a few seconds |
| Robot standing still | heading drifts less than ~1°/s |

* Heading goes the wrong way → `IMU_YAW_TO_COMPASS_SIGN` and `IMU_GYRO_Z_SIGN`
  (board mounted upside down flips both).
* Heading is offset by a constant → `IMU_YAW_OFFSET_DEG`.
* The forward acceleration axis is wrong (speed reads 0 while driving, or jumps
  while turning) → `IMU_ACC_FORWARD_AXIS` / `IMU_ACC_FORWARD_SIGN`.

The default `heading_source: 1` needs no compass calibration: it aligns itself
to the GPS course once the robot moves faster than `align_min_mps` (0.25 m/s).
Indoors, where there is no fix, the heading stays relative — that is expected.

## 2b. Compass calibration (do this once, outdoors)

The QMC5883L inside the GPS module is what tells the robot which way it points
**before it starts moving**. Every chassis has its own iron and its own motor
magnets, so it must be calibrated on the finished robot.

1. Take the robot outside, away from cars, fences and rebar.
2. Web UI → Dashboard → **ปรับเข็มทิศ / Calibrate compass**.
3. Turn the robot slowly through at least one full circle — use the manual
   turn buttons, roughly 20 s per revolution. Two circles is better.
4. Press **บันทึกผล / Finish**. The result is written to the ESP32 flash and
   survives a reboot. "หมุนไม่ครบรอบ / failed" means the turn was too small.
5. Check it: point the robot north, the heading should read ≈0°; east ≈90°.
   A constant offset → `mag_mount_offset_deg`. Heading counting backwards →
   `COMPASS_INVERT` in `config/compass.h`.
6. Set `mag_declination_deg` for your area (magnetic-declination.com; Thailand
   is about +0.5 to +1°). Small, but free accuracy.

If the chip is missing or the field looks disturbed, the UI shows the compass
chip in red/orange and the robot falls back to the BNO085 magnetometer and then
to the GPS course — it keeps working, just with a less certain heading while
standing still.

## 3. Physical constants

Measure and put the same numbers in all three places:

| Value | Files |
|---|---|
| wheel diameter | `config/robot.h: WHEEL_DIAMETER_M`, `robot.yaml` |
| wheel track (centre to centre) | `config/robot.h: WHEEL_TRACK_M`, `robot.yaml`, Settings → PID → `track_m` |

Check the speed scale: drive 10 m in a straight line at 0.25 m/s and compare
the GPS distance shown in the log with the tape measure. If the robot is
consistently slower than commanded, lower `MOTOR_RPM_RATIO`.

## 4. Speed loop

Settings → **PID / ตัวประมาณค่า** → `speed.*`. The loop is feed-forward first,
PI second, so it is usually fine out of the box.

1. Command 0.2 m/s on flat ground (manual ▲ with the power slider at 50 %).
2. Compare `ความเร็ว` with the GPS speed after ~5 s.
   * always short → increase `speed.ki` in steps of 40
   * oscillates in speed (surging) → halve `speed.ki`, then `speed.kp`
3. Grass or a slope should not change the steady speed much — that is what the
   integrator is for.

## 4b. Motor acceleration ramp

Settings → PID → `motor.accel_ramp_s` / `decel_ramp_s` (default 0.6 s / 0.3 s).

| Symptom | Fix |
|---|---|
| Robot lurches when a mission starts, front wheels lift | raise `accel_ramp_s` to 0.8–1.2 |
| Battery voltage dips and the UI shows a brown-out warning on start | raise `accel_ramp_s` |
| Robot feels sluggish reacting to a new heading | lower `accel_ramp_s` to 0.3–0.4 |
| Takes too long to stop | lower `decel_ramp_s` (it is already twice as fast as accel) |

The ramp is on the PWM itself, so it also limits the current spike the two
TB6612 boards see. Watch `มอเตอร์ A/B` current on the dashboard while starting:
with a 0.6 s ramp the peak should be roughly half of what it is with no ramp.

## 4c. Electrical motor model (wheel speed from volts and amps)

Each motor has its own INA226, so the firmware can work the wheel speed out
from the back-EMF: `rpm = ((V_bus − V_diode)·duty − I·R) / Ke`. It needs
three numbers: Ke, R and the diode drop (next section).

**Ke — volts per output rpm.** Datasheet value: 12 V at 100 rpm → `0.12`.
Measure it properly if you want: run the wheel free at a known duty, read the
rail voltage and current on the dashboard, count wheel revolutions for 30 s,
then `Ke = (V·duty − I·R) / rpm`.

**R — total resistance** (winding + TB6612 + wiring). Two ways:
* multimeter across the motor terminals with the motor disconnected, turn the
  shaft slightly and take the lowest reading, then add ~0.3 Ω for the driver;
* or block the wheel, apply a small duty (10 %), read V and I on the dashboard:
  `R = V·duty / I` (back-EMF is zero when the shaft cannot turn).

Put both in Settings → PID → `motor.ke_v_per_rpm` and `motor.resistance_ohm`.

Check it: drive on flat ground and compare `รอบล้อจากไฟฟ้า` (Sensors card) with
the GPS speed converted to rpm. Within ~15 % is normal and good enough.

Wrong `R` shows up as a speed that drops too much under load; wrong `Ke` shows
up as a constant scale error.

`motor.stall_a` (default 2.5 A) is the current level that counts as
"blocked wheel" when there is no back-EMF. Watch the dashboard current with a
wheel held by hand and set it a little under what you see.

## 4d. Series diode drop (schematic draft_5)

Each motor rail has a diode between its INA226 and the TB6612, so the sensor
reads a voltage the motor never actually gets. Both motors have **their own**
setting because they are separate parts.

Datasheet value is usually close enough:

| Part | Typical `Vf` | `R_ohm` |
|---|---|---|
| Schottky SS34 / SR540 / 1N5822 | 0.35 – 0.50 | 0.02 – 0.05 |
| Standard 1N4007 / 1N5408 | 0.80 – 1.00 | 0.05 – 0.10 |

Measure it if you want it exact: with the wheels off the ground and a steady
duty, put a multimeter across the diode (anode to cathode) and read the drop
while the motor runs — that is `Vf` at that current. Do it at two currents and
`R_ohm = ΔV / ΔI`; if you only do one, leave `R_ohm` at 0.

Settings → **มอเตอร์**: `diode_a_vf_v`, `diode_a_r_ohm`, `diode_b_vf_v`,
`diode_b_r_ohm` — applied immediately, no reflash.

Symptom of a wrong value: the two `รอบล้อจากไฟฟ้า` numbers disagree with each
other while the robot drives dead straight. The wheel with the *lower* reading
has the bigger real diode drop.

## 5. Heading loop (driving straight)

Settings → `heading.*`. Tune on the surface the robot will actually work on.

1. Set a heading target and drive 15–20 m: dashboard → manual off, then
   `ros2 topic pub -1 /gps_localize/cmd_move geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 90.0}}'`
2. Behaviour → action:

| Symptom | Fix |
|---|---|
| Slow to come back on line | raise `heading.kp` by ~20 % |
| Weaves left/right (S shape) | lower `heading.kp`, raise `heading.kd` |
| Sits with a constant offset | raise `heading.ki` slightly (0.1 steps) |
| Twitches / nervous steering | raise `heading.tol` (deadband) to 2–3° |
| Turns on the spot too often | raise `HEADING_SPIN_ERR_DEG` (firmware) or `nav.heading_tolerance_deg` |
| Robot stops and turns on the spot too often on the way to a waypoint | raise `control.spin_above_deg`. Below that heading error it already drives and steers together; 180 means it never stops to turn, which is smoother in an open field and worse in a corridor |
| Manual drive curves instead of going straight | it should not any more — manual driving holds the heading. If it still curves, check `MANUAL_HOLD_HEADING` is 1 and that the heading source is not garbage; the hold can only be as straight as the heading it is given |
| After a turn it kicks back the other way before driving on | the heading hold latched before the fused heading had caught up with the robot. It now waits for the yaw rate to be under `MANUAL_HOLD_SETTLE_DPS` for `MANUAL_HOLD_SETTLE_S` first. If a kick is still visible, lengthen the settle - the estimate is lagging further than 0.35 s on this robot |
| It takes a moment to start holding after a turn | that is the settle window doing its job. Shorten `MANUAL_HOLD_SETTLE_S` if it feels slow, but expect the kick back to return as it approaches zero |
| Manual drive fights the turn buttons | lower `MANUAL_HOLD_MAX_PWM`. The hold is a trim, not a controller: it must never out-pull the operator |

Typical good values on grass: `kp 6–12`, `ki 0.2–0.6`, `kd 0.8–2.0`.

## 6. Waypoint behaviour

Settings → **การเดินทาง**:

| Setting | Effect |
|---|---|
| `arrive_radius_m` | how close counts as reached. **Values below 2.0 m are ignored** — the code raises them to that floor, because this receiver cannot resolve a metre. Measured stationary outdoors on 8 satellites at HDOP 1.3: the fix scattered 1.56 m median and 4.86 m worst. A 1 m radius is below the noise, and a mission that drove to the point and never finished is what that looks like |
| `arrive_hold_s` | how long the robot must **stay** inside the radius before it counts, default 1.5 s. This is the half that stops a wider radius from causing early arrivals: with several metres of scatter a single sample can land inside the circle from well outside it. While the timer runs the robot stands still, because at that range the remaining distance is mostly receiver noise and steering on it drives the robot back out of the circle |
| `arrive_sigma_k` | the radius also grows with the fusion's own reported uncertainty, by this factor. On a clean fix the sigma is about 1.5 m and the radius stays where it was set; under trees the sigma grows and the radius grows with it, so the mission still finishes instead of circling a point the robot can no longer resolve |
| `approach_distance_m` / `approach_speed_mps` | how early and how much it slows down |
| `min_satellites` / `max_hdop` | **host-side** gate: below this, navigation blocks and the robot stops. Raising `max_hdop` above 3 lets it drive on bad data |

**There is a second set of GPS thresholds, and they live on the board.** These
two block *navigation*; the board has its own, which decide whether the
receiver's ground *speed* is believed and whether the GPS counts as a working
sensor at all. They used to be compiled in, so changing the pair above moved
only half the robot. They are parameters now, in the **gps** group:

| Board parameter | Default | What it decides |
|---|---|---|
| `gps.min_sats_speed` | 6 | Below this the fused speed ignores GPS and runs on the IMU and the motor model. Strict on purpose: standing still indoors on a 3-satellite fix this receiver reported 7.28 m/s |
| `gps.max_hdop_speed` | 1.5 | Worst HDOP whose ground speed is still believed. Raising it lets indoor noise into the speed estimate, which is what makes the odometry wander |
| `gps.min_sats_fix` | 4 | Below this the GPS is reported as a missing sensor. A deliberately looser, separate question — a fix too rough to take a speed from is still a fix, and 4 is the minimum for a 3D solution. There is no HDOP term here: bad geometry makes a position less precise, not absent |
| `waypoint_timeout_s` | protects against a blocked robot pushing forever |

## 7. Field acceptance test

1. Place 3 waypoints in a triangle 10–20 m apart, save.
2. START. The robot should turn on the spot, drive straight, slow near each
   point and stop within `arrive_radius_m`.
3. While driving, walk out of Wi-Fi range with the phone (or turn Wi-Fi off):
   the robot must stop within about a second, red LED blinking fast.
4. Reconnect: the robot must stay stopped until you press START/RESUME.
5. Press the mushroom switch while driving: motors must stop instantly and the
   red LED goes solid. The motor INA226 readings should fall to ~0 V.
6. Leave the system running for an hour and check that `หน่วยความจำว่าง`
   (free heap) in the footer has not dropped.
