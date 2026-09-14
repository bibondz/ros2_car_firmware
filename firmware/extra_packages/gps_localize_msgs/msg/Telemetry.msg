# Telemetry from the ESP32. Nominally 10 Hz (PUB_TELEMETRY_DIV).
#
# Replaces a packed Float32MultiArray. Named fields mean adding a signal is
# one line here and both sides pick it up - no index to keep in step across
# three files, which is exactly what the old array required and what QC
# check 11 existed to police.

float32 state    # RunState: 0 idle 1 auto 2 manual 3 estop 4 comm_loss
float32 heading_deg    # fused heading, compass degrees (0 = north, CW)
float32 target_heading_deg
float32 heading_error_deg
float32 speed_mps    # fused body speed
float32 target_speed_mps
float32 wheel_rpm_left    # derived from speed + yaw rate (no encoder on this robot)
float32 wheel_rpm_right
float32 wheel_mps_left
float32 wheel_mps_right
float32 roll_rate_dps
float32 pitch_rate_dps
float32 yaw_rate_dps    # counter-clockwise positive
float32 imu_roll_deg
float32 imu_pitch_deg
float32 imu_yaw_deg
float32 pwm_left
float32 pwm_right
float32 battery_v    # pack voltage, read from the motor rail (VCC EMER).
float32 battery_a    # whole-module draw, measured on the battery side
float32 motor_a_v
float32 motor_a_a
float32 motor_b_v
float32 motor_b_a
float32 gps_fix_quality    # 0 none, 1 GPS, 2 DGPS/SBAS
float32 gps_satellites
float32 gps_hdop
float32 gps_speed_mps
float32 gps_course_deg
float32 stop_flags    # bitfield, see STOP_FLAGS
float32 free_heap_kb    # watch this on the web UI: it must stay flat
float32 loop_us    # control loop duration
float32 odom_x    # ESP dead-reckoning, ENU metres
float32 odom_y
float32 mag_heading_deg    # QMC5883L compass in the GPS module, tilt compensated
float32 heading_ref    # what the heading is locked to right now
float32 mag_status    # 0 no compass, 1 ok, 2 magnetic disturbance
float32 mag_calib_state    # 0 idle, 1 capturing, 2 saved, 3 failed (turn was too small)
float32 sensor_health    # bitfield of missing sensors, see SENSOR_HEALTH
float32 speed_source    # what the speed estimate is built from
float32 motor_rpm_left    # back-EMF wheel speed from the INA226 volts + amps
float32 motor_rpm_right
float32 motor_faults    # bitfield: stall / open circuit per motor
float32 fan_v    # fan rail INA226 (schematic draft_5)
float32 fan_a
float32 esp32_rail_v    # ESP32 +5 V rail, health of the processor buck
float32 pack_valid    # 1 = pack voltage is measurable; 0 = motor rail
float32 pos_sigma_m    # 1-sigma position uncertainty from the EKF [m].
float32 pos_anchored    # 1 = the EKF has had at least one fix; 0 = the

# major * 100 + minor, from firmware/config/version.h. Sent in the ordinary
# frame because it is four bytes and needs no new publisher, no string handling
# and no extra type support in the firmware image. 0 from a board too old to
# send it - which is itself the answer to "how old is this board".
float32 firmware_version

# Why the board last restarted, from esp_reset_reason(). This is how a brownout
# is told apart from a crash and from someone pressing reset - they look
# identical in the logs otherwise, and a 3.3 V rail that sags under Wi-Fi bursts
# shows up here as ESP_RST_BROWNOUT and nowhere else.
#   1 POWERON  3 SW  4 PANIC  5 INT_WDT  6 TASK_WDT  7 WDT  8 DEEPSLEEP
#   9 BROWNOUT  10 SDIO
float32 reset_reason

# Seconds since boot. A board that is quietly resetting has a short uptime that
# keeps starting over, which is invisible if you only ever look at live values.
float32 uptime_s

# ESP32 die temperature, degrees C, or NaN when the chip has no usable sensor.
#
# The original ESP32 (WROOM-32) carries a temperature sensor that Espressif
# disabled after early silicon: temperatureRead() still answers, but on most
# boards it returns a fixed value near 53 C no matter how hot the chip is. A
# constant is worse than nothing, because it looks like a reading - so the
# firmware checks whether it MOVES before believing it, and reports NaN when it
# does not.
#
# Note what this is not: the die is not the regulator. The AMS1117 dropping
# 5 V to 3.3 V is the part that actually runs hot, and nothing on the board
# measures it.
float32 esp32_temp_c

# How far round the circle the QMC5883L sweep has actually got, 0.0 to 1.0,
# measured as the fraction of sixteen sectors the field vector has visited.
#
# Sample count is not progress. Standing still with the sensor running piles up
# thousands of samples and calibrates nothing, and that is the failure people
# hit: press start, nudge the robot, press finish, and save a calibration built
# from a fraction of a turn. Zero unless a sweep is running.
float32 mag_calib_progress

# The BNO085's own confidence in its magnetometer: 0 unreliable .. 3 high.
#
# The heading chain refuses to use this sensor below 2. Until this field existed
# that number was only in the USB console's `info` output, so from a browser
# there was no way to tell a magnetometer that merely needs moving from one that
# is broken - both read as imu_magnetometer_missing and stay that way.
float32 imu_accuracy

# 0 idle, 1 learning, 2 saved to the sensor's flash, 3 the sensor refused.
float32 imu_calib_state

# What the BNO085 said to the last calibration command: SH2_OK is 0, anything
# else is a negative code from sh2_err.h (-100 means the IMU never started).
#
# Published because "refused" on its own is not a diagnosis. A sensor that is
# absent, one that is busy, and one that rejected the command outright all look
# identical from a browser, and they need different actions.
float32 imu_calib_result

# Degrees the robot has actually turned since the compass sweep started, from
# the gyro. Signed, so rocking back and forth cancels instead of accumulating.
#
# This is the independent measurement. Everything else the calibration knows
# comes from the magnetometer itself - the very thing being calibrated - so a
# distorted field could both cause a bad result and vouch for it. The gyro
# cannot be fooled that way, and it is what the progress bar is gated on.
float32 mag_turned_deg

# Linear acceleration from the BNO085, in the sensor's own frame, m/s^2.
#
# GRAVITY IS ALREADY REMOVED - this is the SH2_LINEAR_ACCELERATION report, not
# raw accelerometer. A robot standing still reads about zero on all three axes,
# not 9.81 on one of them, and that is the number worth showing: it is what the
# speed estimator actually consumes.
float32 imu_acc_x
float32 imu_acc_y
float32 imu_acc_z

# How many times the BNO085 has reset ITSELF since boot.
#
# The driver notices via wasReset() and re-enables the reports, but each one is
# a gap in the stream - and a gap longer than IMU_STALE_MS makes imu_ok false,
# which reads on the dashboard as the IMU going missing. Measured here: the
# sensor reporting a healthy 3/3 accuracy while "imu_missing" flapped twenty
# times in two minutes. Without this counter there is no way to tell a sensor
# that is resetting from a bus that is dropping data.
float32 imu_resets

# Delivered rate of each BNO085 report, in Hz, measured over a rolling window.
#
# Four reports share one I2C bus - the game rotation vector and the gyroscope at
# 100 Hz, linear acceleration at 50 Hz and the magnetometer-referenced rotation
# vector at 10 Hz. When the bus cannot carry all of them it is the slowest that
# starves first, so the fast reports look perfectly healthy while the heading
# reference disappears. On the dashboard that is indistinguishable from a broken
# magnetometer, and it was mistaken for one.
#
# These existed on the USB console first, which was the wrong place: the robot
# is driven over Wi-Fi and the console needs a cable, so the one number that
# separates "the sensor is bad" from "the bus is full" was unreachable exactly
# when it was needed. Compare each against the rate it was asked for - the
# firmware asks for 100, 100, 50 and 10.
float32 imu_hz_rv
float32 imu_hz_game
float32 imu_hz_gyro
float32 imu_hz_accel

# How far the absolute heading reference disagrees with the IMU's own fused
# yaw, in degrees, signed.
#
# The compass and the BNO085's magnetometer both claim to know north and both
# are fooled by the same things - a motor drawing current, a steel bench, rebar
# in the floor. The IMU's fused yaw comes from the gyroscope, which does not
# care about magnets; it drifts slowly, which is why it needs an absolute
# reference, but over the seconds a disturbance lasts it is the honest one.
#
# Near zero means two independent sources agree, which is worth more than
# either alone. Large means one of them is being disturbed, and the robot is
# refusing the reference rather than steering to it.
float32 heading_innov_deg

# 1 while an absolute heading reference is being refused for disagreeing with
# the IMU. Not a fault: it is the cross-check working. It clears by itself
# when the disturbance passes, and a disagreement that lasts is accepted
# anyway - a gate that can never give up would lock out a correct sensor for
# ever if the gyro really had drifted.
float32 heading_gated

# Wi-Fi signal strength at the robot, in dBm, as a negative number. 0 means the
# board is not associated at all.
#
# It belongs in telemetry for the same reason the loop time and the battery do:
# it is a health number that explains other symptoms. A weak link is the cause
# behind a stuttering web UI, a micro-ROS session that keeps being rebuilt and
# a robot that appears to stop responding at the far end of the garden - and
# without it the only evidence is the failure itself, which points at the
# firmware instead of at the radio. Rough reading: better than -60 is strong,
# -70 is workable, past -80 the link starts dropping frames.
float32 wifi_rssi_dbm
