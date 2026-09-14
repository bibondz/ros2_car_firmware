# Hardware — schematic draft_5

Reference drawing: `SCH_Schematic1_5-draft_5_2026-08-04.pdf` (page `draft_5`).
Still no wheel encoders — GPS + IMU only.

Changes from draft_2, all of them already in the firmware:

| Change | Effect on the code |
|---|---|
| **4th INA226** on the fan rail (`FAN_SENSOR`) | new address `0x45`, published as `fan_v` / `fan_a` |
| **Two LM2596 bucks**: one for the fans, one for the processor | fans can no longer brown out the ESP32 (this was the fix requested) |
| **4 fans** wired individually (`MOTOR_FAN_COOLING_A..D`) | still uncontrolled, always on |
| **Series diode moved**: it now sits *after* the INA226, right before the TB6612 `VM` | the sensor reads the rail **before** the diode, so the motor model subtracts `Vf` — per motor, they can be different parts |
| Component designators renumbered (C1…, R1…) | no effect |

ESP32 pin map, GPS and IMU are unchanged from draft_2.

## ESP32-DevKitC pin map

| Pin | Net (schematic) | Direction | Used for |
|---|---|---|---|
| IO25 | MOTOR A IN1 | out | Motor A (LEFT) direction |
| IO26 | MOTOR A IN2 | out | Motor A direction |
| IO27 | MOTOR A EN | out (PWM) | Motor A speed, TB6612 PWMA+PWMB |
| IO19 | MOTOR B IN1 | out | Motor B (RIGHT) direction |
| IO18 | MOTOR B IN2 | out | Motor B direction |
| IO4 | MOTOR B EN | out (PWM) | Motor B speed |
| IO21 | SDA | I²C | BNO085, 4× INA226, QMC5883L in the GPS module |

**The I²C bus runs at 400 kHz, and that number was re-measured.** It was dropped
to 100 kHz once to stop the BNO085 resetting, together with a longer timeout,
and the timeout was the half that helped. Every judgement of the clock after
that was made while the IMU was refusing to start, so the bus was nearly idle
and 100 kHz looked fine. With the sensor actually streaming, SHTP moves a whole
cargo per report and a few hundred bytes at 100 kHz is tens of milliseconds —
the control loop ran 45 to 161 ms per tick, against 0.56 ms with the sensor
dead. At 400 kHz it is 1 to 8 ms. `ImuBno085::update()` also has a time budget per
tick, which bounds the loop whatever the bus does.

**That budget is 4 ms, and 2 ms was too tight.** Events are drained in arrival
order, so a budget that runs out part way through starves whatever sits at the
back of the queue - which was always the rotation vector. Measured at 2 ms: the
sensor was asked for 200 reports a second and 113 were collected, with the
rotation vector getting 0 to 6 Hz of its 50 while the gyroscope got 65 of 100.
At 4 ms: 168 to 187 of 200, and the rotation vector 43 to 49 of 50. The loop
runs 4.6 to 9.3 ms against a 10 ms period, so this is close to the ceiling - if
control periods start being missed, this is the first number to bring back
down.

**The BNO085 is mounted BACKWARDS.** Its printed X arrow points at the rear of
the robot, so `IMU_ACC_FORWARD_SIGN` is −1 and `IMU_YAW_OFFSET_DEG` is 180.
Those two describe one physical fact and must never disagree — `tools/qc.py`
refuses the combination where they do, because half-fixing it is worse than not
fixing it: a heading squared up while the acceleration still points backwards
looks right and is not.

It was wrong for a long time and nothing reported an error. With the sign at +1
the estimator was fed forward acceleration negated, so it believed the robot was
accelerating backwards whenever it drove forwards. Measured on the floor: a
steady 0.18 m/s forward command produced a fused speed reading anywhere from
−0.36 to +0.34 m/s, negative while the robot was plainly moving forwards. That
was blamed on integration drift for an afternoon; it was the mounting.

**The Wi-Fi radio was browning out the IMU.** The BNO085 reset itself constantly
and lost its calibration each time — 34 resets in 68 seconds at worst — and
every symptom pointed at the sensor or the bus. The comparison that settled it:
`tools_imu/imu_only.cpp` is the same board, the same sensor and the same bus
with **no Wi-Fi**, and it runs for minutes with zero resets at a steady 3/3. An
ESP32 transmit burst at the default 20 dBm is a few hundred milliamps, and on
USB alone — no battery holding the rail up — that browns out a sensor sharing
3V3. `WIFI_TX_POWER` is **13 dBm** for that reason: one reset in 165 seconds
afterwards, and the IMU survived an over-the-air update without a power cycle
for the first time. The proper fix is a battery or bulk capacitance on the
sensor supply, after which the power can go back up.

**Ask the BNO085 for ONE rotation vector, and make it the game one.**
`IMU_REPORT_MODE` in `config/imu.h` decides, and it is deliberately separate
from `HEADING_SOURCE`: those answer different questions — which reports the
sensor publishes, and which absolute references the estimator may use — and
deriving one from the other meant neither could be chosen without disturbing
the other.

It matters more than it sounds, because **both** things this robot reads from
the sensor come from that one report. The heading backbone is the sensor's own
integrated orientation — not a gyro rate integrated in our control loop; the
sensor corrects its own drift against gravity at its own rate, and redoing that
in the loop would be strictly worse because every late cycle is rotation that
silently never happened. Roll and pitch from the same report tilt-compensate the
QMC5883L, so a starved rotation vector quietly makes the compass wrong on any
slope as well.

The game vector is the one to keep: it is that integrated orientation, immune to
magnets because it uses no magnetometer at all, and north comes separately from
the compass and the GPS course — both cross-checked against it. The
magnetometer-referenced vector would give absolute heading directly, but from a
magnetometer sitting next to the motors: the sensor this robot trusts least,
made the backbone of everything.

**The BNO085 runs one fusion, never two.** `MODE_BOTH` asks for the game
rotation vector and the magnetometer-referenced one together and only ever
delivers one of them properly — proved from both directions. With the
magnetometer calibration still running, the game vector got 50-70 Hz and the
magnetometer-referenced one 0.0-0.5 Hz of the 10 it asked for; saving the
calibration to the sensor's flash flipped it, giving `rv` 9.6 Hz and starving
the game vector to 2-7. Halving the game rate changed neither. The gyroscope is
unaffected at 90 Hz throughout, so yaw *rate* is always available. This matters
less than it sounds: the magnetometer-referenced vector is the third absolute
heading source, behind the GPS course and the QMC5883L.
| IO22 | SCL | I²C | 3.3 k pull-ups on the board |
| IO16 | GPS RX (net) | in | ESP RX ← GPS TX |
| IO17 | GPS TX (net) | out | ESP TX → GPS RX |
| IO13 | E-EMER | in | mushroom switch, HIGH = tripped |
| IO14 | RELAY | out | motor rail contactor, HIGH = closed |
| IO13 | ESP CHECK EMER | in | on-board e-stop button (10 k pull-up, LOW = pressed) |
| IO34 | MOTOR A ALER | in | INA226 alert (input-only pin) |
| IO35 | MOTOR B ALER | in | INA226 alert |
| IO32 | PROCESSOR ALER | in | INA226 alert |
| IO2 | LED indicator | out | **RED** LED through 330 Ω |
| IO33, IO23, IO5, IO12, IO15, IO0 | — | — | free |

Firmware copy of this table: `firmware/config/pins.h`.

## Motor drivers

Each TB6612FNG has **both half-bridges wired in parallel** (AIN1+BIN1,
AIN2+BIN2, PWMA+PWMB, A01+B01, A02+B02) so one board drives one motor at
double the current. `STBY` is tied to +3.3 V. The firmware therefore only
drives 3 pins per motor.

* Motor A = LEFT wheel, Motor B = RIGHT wheel.
* If a wheel turns the wrong way, flip `MOTOR_A_INVERT` / `MOTOR_B_INVERT`
  in `firmware/config/motor.h` (no rewiring needed).
* PWM: 20 kHz, 10 bit, ESP32 LEDC channels 0 and 1.

## Power sensing — 4× INA226

```
VCC BATT ─ switch ─ VCC ON MODULE ─┬─[FAN_SENSOR]───── VCC FAN ───── LM2596 ─ +5V FAN ─ diode ─ 4× fan
                                   └─[PROCESSOR_SENSOR] VCC PROCESSOR LM2596 ─ +5V ─ ESP32

VCC BATT ─ fuse ─ mushroom ─ relay ─ VCC EMER ─┬─[MOTOR_A_SENSOR]─ VCC MOTOR A ─▶|─ TB6612 VM
                                               └─[MOTOR_B_SENSOR]─ VCC MOTOR B ─▶|─ TB6612 VM
```

| Sensor | Measures | Default address | Shunt |
|---|---|---|---|
| PROCESSOR_SENSOR | the rail feeding the ESP32 — **3V3 since the board moved onto its own buck**, measured 3.42 V / 172 mA for the ESP32 plus every sensor. Was the 5 V rail, shared with the relay coil. | 0x41 | 0.1 Ω / 0.8 A |
| MOTOR_A_SENSOR | VCC EMER → motor A rail, **before the diode** | 0x40 | 0.01 Ω / 3 A |
| MOTOR_B_SENSOR | VCC EMER → motor B rail, **before the diode** | 0x44 | 0.01 Ω / 3 A |
| FAN_SENSOR | VCC ON MODULE → fan buck (draft_5, new) | 0x45 | 0.1 Ω / 1.5 A |

INA226 address jumpers (A1,A0): GND,GND `0x40` · GND,VS `0x41` · VS,GND `0x44` ·
VS,VS `0x45`. `FAN_SENSOR`'s ALERT pin is not wired — the firmware does not use
the alert outputs, it polls all four sensors at 5 Hz.

### The series diode (`MOTOR_A_DIODE` / `MOTOR_B_DIODE`)

Each motor rail has a diode between its sensor and the TB6612 `VM` pin, with
the 1000 µF + 0.1 µF bulk caps after it. The sensor therefore reads the voltage
**before** the diode and the motor only ever gets

```
V_driver = V_sensor − (Vf + I × R_slope)
```

The back-EMF wheel-speed model subtracts exactly that, so `Vf` must match the
part you fitted — and the two motors have separate settings because they are
separate diodes:

```c
// firmware/config/motor.h        (or Settings → มอเตอร์ on the web UI)
#define MOTOR_A_DIODE_VF_V      0.181f  // MEASURED on this board, left
#define MOTOR_A_DIODE_R_OHM     0.03f   // slope, 0 = flat drop
#define MOTOR_B_DIODE_VF_V      0.168f  // MEASURED on this board, right
#define MOTOR_B_DIODE_R_OHM     0.03f
```

**Set the A0/A1 jumpers before power-up** or change `INA226_ADDR_*` in
`power.h`. Stock CJMCU-226 boards ship with a 0.1 Ω shunt (max 0.8 A) — fine
for the processor and fan rails, too small for the motors. Fit 0.01 Ω on the
two motor sensors and keep `INA226_SHUNT_MOTOR_OHM` in sync.

Battery protection lives in `power.h`, as 3S LiPo tiers: warn at 11.7 V
(3.90 V/cell), soft stop at 11.4 V, and hard cut at 11.199 V held for 3 s. A cell
is damaged below about 3.0 V/cell (9.0 V pack), so these sit well clear of it.

The warning and the soft stop follow the voltage back up on their own. **The
hard cut does not: it is cleared by switching the robot off and starting it
again, and by nothing else.** A pack at the cutoff sags under load and springs
back as soon as the load comes off, so a recovery threshold would only make the
robot stop, un-latch, drive, sag and stop again while draining the cells past
the point where they take damage. No voltage means "this pack is fine now" -
only a person deciding to charge or swap it does.

The diode drops above are measured, not datasheet figures - at ~12 V a 1.8 V
drop is 15% of the rail, and the back-EMF wheel-speed model uses it, so a
guess makes the derived speed read high whenever GPS is unavailable.

Check the addresses on the bench:

```bash
# ESP32 serial monitor prints "[INA226] ... missing" for each sensor it cannot see
pio device monitor -b 115200
```

## Board notes

### 1. Two LEDs instead of one

This is the one item the drawing still shows as a single LED:

| LED | Wiring | Meaning |
|---|---|---|
| **RED** | IO2 → 330 Ω → LED → GND (as drawn) | solid = emergency stop / E-emergency latched · fast blink = link or command lost · slow blink = battery cutoff · short blip every 1.5 s = healthy |
| **GREEN** | +5 V (after the ON MODULE switch) → 330 Ω → LED → GND, **no GPIO** | module is powered / switch is ON. Deliberately not driven by firmware so it still lights if the ESP32 hangs |

### 2. Cooling fans — done in draft_5

The 4 fans now run from their own LM2596 (`FAN_BUCK`) fed through `FAN_SENSOR`,
separate from the processor buck, so fan inrush cannot brown out the ESP32.
`FAN_DIODE` sits on the buck output.

The fans run continuously — the firmware neither switches nor needs to switch
them, it only reports the rail on the dashboard. If you later add a MOSFET, set
`PIN_FAN_CTRL` in `pins.h` to that GPIO (IO33 is free).

## Emergency stop wiring

```
VCC BATT ─ 10 A fuse ─ relay contacts ──► EMER VCC ──► motor rails
                            ▲                     (INA226 motor A/B measure here)
                            │
                       relay coil ── MOSFET ◄── IO14  (firmware drives HIGH to close)
                                        │
                                    pull-down     so the rails stay dead through
                                        │         reset, boot and any crash
                                       GND

SW3 (mushroom, latching) ──► IO13   HIGH = tripped; firmware opens the contactor
```

* `E-EMER` (IO13) tells the firmware whether the mushroom switch is tripped.
* `RELAY` (IO14) is an **output**: the firmware drives it high to close the
  main contactor and power the motor rails. Drive it through a MOSFET, never
  straight from the GPIO - a 5 V coil wants 70-100 mA and the pin is good for
  about 20 mA - and give the gate a **pull-down**. During reset, boot and any
  crash the pin is high impedance, so the resistor decides the relay state: a
  pull-down leaves the rails dead until firmware powers them deliberately,
  where a pull-up would close the contactor before any code has run.
  `E_EMER_ACTIVE_LOW` in `safety.h` is set to 1 — **verify the real
  polarity on the bench** with a multimeter before the first drive test and
  change the define if the LED lights the wrong way round.
* `SW4` (IO13) is the small on-board e-stop button, active LOW.
* Because the motor rails sit behind the relay, `MOTOR_A_SENSOR`/`MOTOR_B_SENSOR`
  read ~0 V when the emergency loop is open — a second, electrical proof that
  power really was cut.

## Mechanical / calibration constants

| Constant | Where | Default | Note |
|---|---|---|---|
| Wheel diameter | `WHEEL_DIAMETER_M`, `robot.yaml` | 0.081 m | 81 mm wheels |
| Wheel track | `WHEEL_TRACK_M`, `robot.yaml`, `pid.yaml:estimator.track_m` | 0.300 m | **measure your chassis and update all three** |
| Max speed | `ROBOT_MAX_SPEED_MPS` | ≈0.42 m/s | derived from 100 rpm × Ø81 mm |
| IMU mounting | `IMU_ACC_FORWARD_AXIS`, `IMU_YAW_TO_COMPASS_SIGN` | X forward, chip up | see docs/tuning.md |

## GPS module and its compass

GEP-M10-DQ: UART for position, and the **QMC5883L magnetometer inside the
module is used** (I²C address `0x0D`, on the same bus as everything else).

That compass is what gives the robot an absolute heading while it is standing
still — the GPS course over ground only exists once it is moving, and the
BNO085 game rotation vector is relative. It sits on the antenna mast, away from
the motors, which makes it the better of the two magnetometers on the robot.
The readings are tilt-compensated with roll/pitch from the BNO085 and rejected
when the field strength jumps (motor current, steel fence): see
`firmware/lib/compass/compass_qmc5883.h` and calibrate once with the web UI
button (docs/tuning.md §2b). The calibration is stored in the ESP32 flash.

Set `USE_QMC5883L 0` in `compass.h` if your module has no compass — the
firmware then falls back to the BNO085 magnetometer and the GPS course.

The DPS310 barometer in the module is not used.

Baud rate is auto-detected at boot (115200 → 38400 → 9600 → 57600). For the
best results configure the module once with u-center: 5 Hz navigation rate,
GGA + RMC + VTG enabled, GSV/GSA/GLL disabled.
