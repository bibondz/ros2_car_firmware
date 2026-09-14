# Calibration

Every calibration this robot has is on one page of the web UI —
**ปรับเทียบ / Calibration**. Nothing here needs a USB cable, a terminal or a
`ros2` command.

Open the web UI (see `docs/running.md`), press the **ปรับเทียบ / Calibration**
tab, and work down it. Steps 1 and 2 are enough to drive. Steps 3 and 4 are
accuracy, and can wait until the robot is behaving.

---

## 1 · Compass — QMC5883L

**What it is for.** An absolute heading while standing still. GPS course only
exists while moving, and the BNO085's game rotation vector is relative, so
without this the robot does not know which way it is pointing until it has
driven a few metres.

**How to do it.**

1. Put the robot where it will actually work: battery in place, lid on, away
   from steel benches and reinforced concrete. What is being measured is the
   magnetic distortion *of this robot*, so anything that will not be there later
   should not be there now.
2. Press **เริ่ม / Start**.
3. Turn the robot slowly through one full circle — about 20 seconds. Use the
   arrow buttons, or pick it up and turn it.
4. Press **บันทึก / Finish**.

**The progress bar is real.** It only moves while the robot actually turns.
Sample count would not be: standing still with the sensor running piles up
thousands of samples and calibrates nothing, which is exactly the failure this
replaced — press start, nudge, press finish, and save a calibration built from a
fraction of a turn.

The bar turns green at three quarters, which is the point where the board will
accept the result. Below that it refuses and says so, and nothing is stored.

**If the compass dies after calibrating, press ล้างค่า / Clear.** This is not a
hypothetical. The magnetic-disturbance check only runs *once a calibration is
stored*, so a bad calibration does not merely tilt the heading — it switches on
a test that then rejects every reading, and a healthy compass goes completely
dead with the heading pinned at zero. It happened on this robot: the stored
reference came out at 1057 against a real field of 1962, and every frame was
rejected from that moment on. Clear puts it back to raw readings, which work.

---

## 2 · IMU — BNO085

**What it is for.** The third heading source, used when the compass is missing
or disturbed. The heading chain refuses this sensor entirely below accuracy 2.

**How to do it.**

1. Press **เริ่ม / Start**.
2. Move the robot in slow figure-of-eights — roll it, tip it, turn it — **away
   from the motors and the battery**. Both are magnets as far as this sensor is
   concerned, and it sits next to them.
3. Watch the four pips. They go amber at 1, green at 2. Keep going until 2.
4. Press **บันทึกลงเซ็นเซอร์ / Save**.

**Why there is a Start button at all.** The BNO085 calibrates itself from
motion, but stops once it has a saved record. If the robot changes around it — a
battery fitted beside it, the sensor remounted, a magnet moved — that record is
wrong and the accuracy it reports never recovers on its own. Start tells it to
learn again.

**Why Save matters.** Without it the figure-of-eight has to be repeated after
every power cycle. Save writes the result into the sensor's own flash.

**You should not have to do this twice.** The firmware saves the calibration
into the sensor's own flash the first time it reaches 3/3, so a reset — and the
BNO085 does reset itself, see `docs/hardware.md` — no longer loses it. Before
that, a brown-out took a steady 3/3 down to 0 and it stayed there until somebody
walked over and waved the robot about again.

**`imu_magnetometer_missing` is not a fault.** It means accuracy is below 2 —
the sensor needs moving, not replacing. Before this page existed that number
lived only in the USB console's `info` output, so from a browser a magnetometer
that needed waving about and one that was broken looked identical.

**More often it means the report is not arriving, which is not the same thing.**
The flag goes true when the magnetometer-referenced heading is below accuracy 2
**or** stale, and those are different faults with different fixes. A sensor
reading a steady 3/3 whose report is not arriving does not need waving about;
nothing about the magnetometer is wrong.

The **IMU reports** row on the dashboard shows the delivered rate of each report
against the rate asked for, and separates the two cases outright.

**On this robot the BNO085's magnetometer is not in use at all**, and that is a
choice rather than a fault. `IMU_REPORT_MODE` asks the sensor for the game
rotation vector only - see `docs/hardware.md` for why - and the
magnetometer-referenced report is never enabled, so `rv` reads 0 Hz and
`imu_magnetometer_missing` is no longer raised. North comes from the QMC5883L
and the GPS course, both cross-checked against the IMU.

The accuracy figure follows whichever rotation vector is running, so it now
reports the accelerometer-and-gyro fusion the sensor actually performs. It
briefly read a permanent 0/3 after the report mode changed, because its only
writer lived in the handler for the report that had just been switched off -
which also made the automatic calibration save, waiting for 3/3, unreachable.

`rv` sitting near zero is therefore expected: **the BNO085 runs one fusion, never two.** `MODE_BOTH` asks for the game
rotation vector and the magnetometer-referenced one together and only ever
delivers one of them properly — proved from both directions, and no report rate
changes it. `docs/hardware.md` has the measurements. It costs little, because
this is the third absolute heading source behind the GPS course and the
QMC5883L, and the gyroscope keeps running at 90 Hz throughout so yaw *rate* is
always available.

The standalone tool measures it without ROS or Wi-Fi in the way: flash
`pio run -e imu_only -t upload`, open the serial monitor and press **`s`**. It
tries every combination of I²C clock (100 / 200 / 400 kHz) against every
magnetometer report rate (10 / 25 / 50 / 100 Hz) under the firmware's real bus
load and prints what each one actually delivers.

The staleness allowance for this one report is ten of its own report periods,
not the 500 ms that applies to the fast reports. Judged against 500 ms a 10 Hz
report was called missing after five consecutive misses, which is an ordinary
hiccup — that is what made the flag flap while nothing was wrong.

---

## 3 · Wheels and track

Two numbers, both measured with a tape, both of which decide what a metre and a
degree mean to every control loop.

| Parameter | What to measure |
|---|---|
| `robot.wheel_diameter` | The wheel **loaded**, with the robot's weight on it. A soft tyre is smaller than the one in the catalogue. |
| `robot.track_width` | Centre to centre between the two wheel contact patches. |

Press **แก้ค่าล้อ / Edit these →** to jump to the fields. Both are refused while
the robot is moving, because changing them mid-drive silently redefines every
speed and turn rate the control loops are working with.

---

## 4 · Motor model

**What it is for.** With no GPS the robot works out its speed from motor volts
and amps alone — back-EMF. These numbers are what make that guess right, and the
shipped defaults came from a datasheet rather than from this robot.

The Calibration tab shows everything needed while the wheels turn: rail volts
and amps per motor, the PWM going in, the rpm the model computes, and the rpm
the fused estimate believes.

| Parameter | How to arrive at it |
|---|---|
| `motor.a.diode_vf`, `motor.b.diode_vf` | **Measure it.** Put a meter across each series diode with the motor running. Not a tuning knob. The default shipped at 0.45 V from a datasheet and measures **0.181 V and 0.168 V** on this robot. Those were written down as 1.81 and 1.90 for a while, which is ten times too large and not a voltage any diode produces - 1.8 V is an LED, not a rectifier. A wrong drop goes straight into the back-EMF speed, and it is plausible enough to survive review, which is exactly why it has to be measured rather than remembered. |
| `motor.resistance` | Winding + driver + wiring. Measure across a stalled motor at a known duty: `R = V / I`. |
| `motor.ke` | Back-EMF constant. Spin the motor at a known rpm and measure the open-circuit volts. |
| `motor.stall_a` | The current that means stalled rather than merely loaded. A stall reported at low current is this threshold, not the motor. |

If the model rpm and the fused rpm disagree, adjust `motor.ke` first, then
`motor.resistance`.

---

## 5 · Zero the position

Not really a calibration — it clears the distance travelled so that where the
robot is standing becomes 0, 0. It is on this page because it is the other thing
people come here looking for.

---

## What is *not* calibrated

- **GPS.** There is nothing to calibrate. The GEP-M10 outputs a position
  solution, not carrier phase, so there is no raw observable to correct. The
  ~1.5–2.5 m floor is the receiver, not a setting.
- **Pin assignments.** A GPIO number describes how the board is soldered, not a
  setting. `pins.h` stays compiled in and is shown read-only.

## Doing it without a browser

The USB console does the compass, and reports the IMU's accuracy:

```bash
cd firmware && pio device monitor -b 115200
```

```
help              # the command list
info              # sensor state, including the BNO085 accuracy 0-3
compass           # the stored calibration
compass clear     # forget it, back to raw
```

Note that **opening the serial port resets the board**, so anything in progress
is lost the moment you connect.
