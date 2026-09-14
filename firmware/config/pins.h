/*
 * Copyright 2026 Phuthiphong Wongchantib
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Original author / contributor:
 * Phuthiphong Wongchantib
 */
/**
 * @file pins.h
 * @brief GPIO map - schematic "SCH_Schematic1_2-draft_2_2026-07-28".
 *
 * EDIT THIS FILE ONLY IF YOU REWIRE THE BOARD.
 * Everything else (speeds, gains, timeouts) lives in the other config files.
 *
 * Free pins on this build: IO33, IO23, IO5, IO12, IO15, IO0.
 */
#ifndef PINS_H
#define PINS_H

//------------------------- motor drivers ---------------------------------//
// Two TB6612FNG boards, both half-bridges paralleled on each board, so one
// board drives one motor with 3 GPIOs. STBY is tied to +3.3 V on the PCB.
// MOTOR A = LEFT wheel, MOTOR B = RIGHT wheel.
//
// THESE WERE THE OTHER WAY ROUND, and the robot told us so: a forward command
// turned it right, and a left command drove it forward. Measured PWM was equal
// on both sides for forward, so the mixer was never at fault. Solving both
// observations against all eight combinations of swapped/not-swapped and either
// motor polarity gives exactly one answer - the two channels were SWAPPED and
// one was REVERSED. A single reversed motor cannot produce it: invert one and
// "left" drives BACKWARD instead of forward. Two predictions from that model
// were then confirmed on the robot before anything was changed here: pressing
// RIGHT drove it backward, and pressing BACKWARD turned it left.
//
// So the LEFT wheel is on IO19/IO18/IO4 and the RIGHT wheel on IO25/IO26/IO27,
// the right one wired with reversed polarity - hence MOTOR_B_INVERT in motor.h.
#define MOTOR_A_IN1             19        // IO19  (left wheel)
#define MOTOR_A_IN2             18        // IO18
#define MOTOR_A_EN              4         // IO4,  PWMA+PWMB
#define MOTOR_B_IN1             25        // IO25  (right wheel)
#define MOTOR_B_IN2             26        // IO26
#define MOTOR_B_EN              27        // IO27, PWMA+PWMB

//--------------------------- I2C bus -------------------------------------//
// Shared by the BNO085, the 3x INA226 and the QMC5883L inside the GPS module.
#define SDA_PIN                 21        // IO21
#define SCL_PIN                 22        // IO22
// 400 kHz. Dropping to 100 kHz for noise immunity was TRIED AND WAS WORSE:
// the mean control-loop time went from 5,699 us to 48,958 us and telemetry fell
// from ~10 Hz to 1.6 Hz. The bandwidth is genuinely needed - imu.update()
// services up to 8 SHTP transactions every single cycle, and at a quarter of the
// clock they no longer fit in the 10 ms period. Slower edges would have been
// more robust per bit, but only by making the bus the bottleneck instead.
// Noise immunity has to come from the wiring, not from the clock.
// 100 kHz, not 400.
//
// The evidence that forced this: at a 15 ms timeout the BNO085 reset itself
// 0.65 times a second, because reads were being cut off mid-transfer. Raising
// the timeout to 60 ms stopped the resets - and the loop then spent the time
// instead, with a 95th percentile of 128 ms against a 10 ms period. Both
// symptoms are the same underlying fact: reads to this sensor frequently do not
// complete in time at 400 kHz.
//
// A slower clock is more tolerant of long wiring, weak pull-ups and a sensor
// that clock-stretches, and this bus carries seven devices. 100 kHz is four
// times the margin on every edge; the traffic here is a few hundred bytes per
// control cycle, which 100 kHz carries comfortably.
//
// An earlier note in this file said 100 kHz "was tried and was worse". It was -
// but that was measured before the timeout was understood, with a 15 ms limit
// that made every slow transfer look like a bus fault. Retested with the reset
// counter and the loop percentiles visible, which is the first time either
// could actually be judged.
//
// AND THEN MEASURED AGAIN, WITH THE IMU ACTUALLY ALIVE, WHICH CHANGED THE
// ANSWER. Every earlier judgement of the bus clock was made while the BNO085
// was refusing to start, so the bus was nearly idle and 100 kHz looked fine.
// With the sensor streaming, the control loop ran 45 to 161 ms per tick - and
// 0.56 ms when the sensor was dead. The difference is the sensor's own traffic:
// SHTP moves a whole cargo per report, so a few hundred bytes at 100 kHz is
// tens of milliseconds, every tick.
//
// 400 kHz cuts that by four. The reason it was blamed before was the 15 ms
// timeout, and that is now 60 ms - the two changes were made together and only
// one of them was the fix. The time budget in ImuBno085::update() is the
// backstop either way: it bounds the loop no matter what the bus does.
#define I2C_CLOCK_HZ            400000
// A transaction that takes longer than this is abandoned. The sensor reads run
// inside the control loop, so an unbounded wait is a stalled robot rather than
// merely a missing reading.
//
// 5 ms WAS TOO TIGHT AND IT KILLED THE IMU. A BNO085 talks SHTP: a single
// getSensorEvent() is a header read followed by a payload read, and the sensor
// clock-stretches while it assembles a packet - so its transactions are far
// longer than an INA226's register read. With 5 ms the events stopped arriving
// entirely, imu_missing stayed set on every frame, yaw_rate read a constant
// 0.000, and the speed estimate fell to its worst rung, "motor model only",
// with nothing on the dashboard saying the timeout had caused it.
//
// 15 ms is the compromise: still roughly 50x a healthy 400 kHz transaction, so
// a bus disturbed by motor noise is abandoned quickly, but long enough that the
// IMU's own pauses are not mistaken for a fault. The measured worst case is
// bounded by this times the transactions in one cycle.
#define I2C_TIMEOUT_MS          60
// A separate, generous timeout for start-up only.
//
// 60 ms, raised from 15 after measuring what 15 was costing.
//
// THE BNO085 CLOCK-STRETCHES, and a timeout that fires mid-transfer leaves it
// looking at a truncated transaction - which it answers by resetting itself.
// Measured on this robot: with 15 ms the sensor reset 0.65 times per SECOND,
// 39 resets in 40 seconds, while reporting a perfectly healthy 3/3 accuracy
// throughout. Every reset is a gap in the report stream, and a gap longer than
// IMU_STALE_MS reads on the dashboard as the IMU going missing - which is
// exactly the "fine, not fine" flapping, 82% of samples showing missing on a
// sensor that was working the whole time.
//
// The standalone imu_only tool, which uses 250 ms, resets once or twice a
// MINUTE. Same sensor, same wiring, same bus - the timeout was the difference.
//
// The old reasoning below is still correct, which is why this is 60 and not
// 250: the loop has to be protected. 60 ms is four control periods, so a truly
// dead bus still costs far less than the 1.07 s stall that motivated the short
// value, while leaving the BNO085 room to finish a transfer it has started.
//
// I2C_TIMEOUT_MS is short on purpose: every sensor read happens inside the
// 100 Hz control loop, so a bus that stops answering must fail fast. None of
// that applies during setup(), where nothing is being steered and there is no
// loop to protect - and the BNO085's SHTP handshake clock-stretches through a
// reset that can take far longer than one control period. Holding the loop
// budget over start-up bought nothing and cost the IMU: the bus scan listed
// 0x4A, and begin() failed anyway.
#define I2C_TIMEOUT_INIT_MS     250
// Why 5 and not 20: with 20 ms the worst observed control-loop stall came out at
// 170 ms, which is almost exactly 8 x 20. That is the signature of ONE cycle in
// which every transaction timed out - imu.update() alone services up to 8 SHTP
// events, each its own transaction. The timeout does not stop the bus being
// disturbed by a 2 A reversal; it only bounds how long the loop waits, so the
// bound has to be small enough to survive being hit eight times over. A healthy
// 400 kHz transaction is a few hundred microseconds, so 5 ms is still ~25x
// headroom, and the worst case falls to about 40 ms.

//---------------------------- GPS UART -----------------------------------//
#define PIN_GPS_RX              16        // IO16 <- GPS TX
#define PIN_GPS_TX              17        // IO17 -> GPS RX

//------------------------ safety and indicators --------------------------//
// IO13 is an emergency INPUT and IO14 is the relay OUTPUT that powers the motor
// rails. These were the other way round in the firmware - IO14 was read as an
// "emergency relay sense" and nothing ever drove a relay at all. The coil was
// therefore never energized, the relay never closed, the motor rails stayed
// dead, and the pack (which sits behind the relay) read 0 V and could not be
// measured. The symptom on the bench was that the relay never clicked.
//
// Schematic net "ESP CHECK EMER": SW2 to ground with R2, a 10K pull-up to
// +3.3 V. So the line idles HIGH and the switch pulls it LOW - it is active
// LOW, not active high. Configured with the internal pull-up as well, so the
// board reads correctly whether or not R2 is populated: unpressed is HIGH
// either way, pressed is LOW either way.
#define PIN_E_EMER              13        // IO13 <- SW2 emergency switch, LOW = tripped
#define PIN_ESP_EMER_SW         -1        // no second button in this build

// Schematic net "E-EMER": IO14 to the relay module's EN pin. Active HIGH - the
// pin is driven high to close the relay, and the firmware holds it low until it
// has decided the robot is safe to power.
//
// The schematic shows R1 as a 10K PULL-UP on this line. Do not fit it. EN is
// active high, so a pull-up closes the contactor whenever the ESP32 is not
// actively holding it low - which is exactly during reset, boot and any crash,
// the three moments the motor rails must be dead. Fit a pull-DOWN instead. It
// costs nothing: the coil current is what draws power, and a MOSFET gate held
// high draws essentially none.
//
// Drive it through a MOSFET or transistor, never straight from the GPIO: a 5 V
// coil wants 70-100 mA and an ESP32 pin is good for about 20 mA.
#define PIN_RELAY_MAIN          14        // IO14 -> motor rail relay coil
#define RELAY_MAIN_ACTIVE_LOW   0         // 0 = drive HIGH to close
// Input polarity of the two emergency inputs: 1 = the pin reads LOW while the
// emergency is ASSERTED, 0 = it reads HIGH. safety_manager.h tests these with
// #if, so an undefined name would silently mean "active high" there while
// still failing to compile in the host tests.
#define E_EMER_ACTIVE_LOW       1         // R2 pull-up idles high; SW2 pulls it low
#define ESP_EMER_SW_ACTIVE_LOW  0         // button drives the pin high when pressed
#define PIN_LED_RED             2         // IO2 through 330R: e-stop / link lost
#define PIN_LED_GREEN           -1        // hardwired to 5 V, firmware never drives it
#define PIN_FAN_CTRL            -1        // fans run from their own buck; set a GPIO only
                                          // if you add a MOSFET for fan switching

//--------------------------- INA226 alerts -------------------------------//
#define INA226_ALERT_MOTOR_A    34        // IO34 (input only)
#define INA226_ALERT_MOTOR_B    35        // IO35 (input only)
#define INA226_ALERT_PROCESSOR  32        // IO32

#endif // PINS_H
