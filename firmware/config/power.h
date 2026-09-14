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
 * @file power.h
 * @brief 4x INA226 voltage/current sensors and battery protection.
 *
 * Wired to schematic draft_5 (SCH_Schematic1_5-draft_5_2026-08-04):
 *
 *   VCC BATT ─ switch ─ VCC ON MODULE ─┬─ [FAN_SENSOR] ─ VCC FAN ─ LM2596 ─ +5V FAN ─ diode ─ 4x fan
 *                                      └─ [PROCESSOR_SENSOR] ─ VCC PROCESSOR ─ LM2596 ─ +5V ─ ESP32
 *
 *   VCC BATT ─ fuse ─ mushroom SW ─ relay ─ VCC EMER ─┬─ [MOTOR_A_SENSOR] ─ VCC MOTOR A ─ diode ─ TB6612 VM
 *                                                     └─ [MOTOR_B_SENSOR] ─ VCC MOTOR B ─ diode ─ TB6612 VM
 *
 * WHAT EACH SENSOR ACTUALLY MEASURES
 * An INA226 measures bus voltage and shunt current at points that need not be
 * the same node, and on this board two of them are not. Getting this wrong
 * once cost us a robot that latched into e-stop three seconds after boot, so
 * it is spelled out:
 *
 *   PROCESSOR 0x41  voltage : the ESP32 +5 V rail, AFTER the LM2596
 *                   current : from the battery side, so it includes the buck
 *                   => V and I are different nodes. V*I is NOT a power figure.
 *                   => this is NOT the pack voltage. It reads about 5 V.
 *
 *   FAN       0x45  voltage : the fan +5 V rail, after its own LM2596
 *                   current : from source, including that buck
 *
 *   MOTOR A   0x40  voltage : VCC EMER, the pack behind the fuse, mushroom
 *   MOTOR B   0x44            switch and relay - so this IS the pack voltage
 *                   current : source -> driver -> motor
 *                   The series diode sits AFTER the sensor, so the driver sees
 *                       V_driver = V_sensor - (VF + I * R_diode)
 *                   which is what the back-EMF wheel-speed model uses.
 *
 * PACK VOLTAGE therefore comes from the motor rails, not from the processor
 * sensor. When the mushroom switch or the relay is open those rails read near
 * zero: that means "cannot measure", not "flat battery", and the firmware
 * marks the reading invalid rather than tripping the low-battery latch.
 *
 * SET THE A0/A1 JUMPERS on each module to match the addresses below before
 * powering up. The serial monitor prints "[INA226] ... missing" for every
 * sensor it cannot find. Address table of the INA226 (A1,A0 -> address):
 *      GND,GND 0x40   GND,VS 0x41   GND,SDA 0x42   GND,SCL 0x43
 *      VS,GND  0x44   VS,VS  0x45   VS,SDA  0x46   VS,SCL  0x47
 */
#ifndef POWER_CONFIG_H
#define POWER_CONFIG_H

//---------------------------- addresses ----------------------------------//
// MEASURED on the built robot with a live 12.14 V pack and the contactor
// closed, not taken from the schematic:
//
//     0x40  12.142 V           -> a motor rail, at pack voltage
//     0x41   5.178 V, 0.89 A   -> the processor rail, whole-module draw
//     0x44  12.141 V           -> the other motor rail
//     0x45   5.174 V           -> the fan rail
//
// PROCESSOR and MOTOR_A were the wrong way round. The pack voltage still came
// out right by luck: packVoltage() ignores any motor rail below
// PACK_RAIL_MIN_VALID_V, and 5.18 V fails that test, so it fell through to the
// one genuine motor rail. What was NOT right is motor A's current - the model
// was being fed the processor's 0.89 A as if it were motor A's, which would
// have corrupted the left wheel's back-EMF speed estimate the moment the robot
// drove. A silent error in a number that looks plausible.
//
// A/B is still unproven: 0x40 and 0x44 are both motor rails and read the same
// voltage with no motors fitted. Which one is A is settled by driving one motor
// and seeing whose current rises - see E16.
#define INA226_ADDR_PROCESSOR   0x41      // measured: the ESP32 +5 V rail
// These follow the driver boards, so swapping the motor pins swaps these too.
// Proved by driving ONE wheel at a time - mixing forward and turn so the other
// side lands on exactly zero PWM: the board on IO25/26/27 draws on 0x40 and the
// board on IO19/18/4 draws on 0x44. Since IO19/18/4 is the LEFT wheel, 0x44 is
// motor A. Getting this wrong is not cosmetic and not visible: the back-EMF
// model would be fed each wheel's current from the OTHER wheel - a wrong number
// that looks entirely plausible, which has already happened once on this robot.
#define INA226_ADDR_MOTOR_A     0x44      // left wheel, the board on IO19/18/4
#define INA226_ADDR_MOTOR_B     0x40      // right wheel, the board on IO25/26/27
#define INA226_ADDR_FAN         0x45      // VCC ON MODULE -> fan buck   (draft_5, new)

//------------------------------ shunts -----------------------------------//
// Stock CJMCU-226 modules ship with 0.1 ohm (max ~0.8 A): fine for the
// processor and fan rails, too small for the motors. Fit 0.01 ohm on the two
// motor sensors and keep these numbers in step with the parts you fitted.
#define INA226_SHUNT_PROCESSOR_OHM   0.100f
#define INA226_MAXA_PROCESSOR        0.8f
#define INA226_SHUNT_FAN_OHM         0.100f
#define INA226_MAXA_FAN              1.5f      // 4 fans in parallel
#define INA226_SHUNT_MOTOR_OHM       0.010f
// 3 A: the module's own limit, not a guess. max_amp sets the calibration
// register and therefore the current LSB, so overstating it coarsens every
// reading - and the back-EMF wheel-speed model consumes that current, so a
// wrong range shows up as a wrong speed whenever GPS is unavailable.
// At 3 A across the 0.01 ohm shunt the drop is 30 mV, well inside the
// INA226's +-81.92 mV input range.
#define INA226_MAXA_MOTOR            3.0f

//------------------------ battery protection -----------------------------//
// 3S LiPo: 12.6 V full, 11.1 V nominal, 9.9 V empty (3.3 V per cell).
// Measured on the processor rail, which sits directly on VCC ON MODULE.
// 3S LiPo. A cell is damaged below about 3.00 V (9.0 V pack); the usual working
// floor is 3.3-3.5 V/cell. These tiers sit above that on purpose - they trade a
// little runtime for pack life.
// PER CELL, then multiplied by the cell count. The thresholds used to be
// absolute volts for a 3S pack, so fitting a 2S or 4S battery meant every tier
// was wrong at once - and wrong in the dangerous direction for 4S, where a 3S
// cutoff of 10.7 V is 2.68 V/cell and the pack would be ruined long before the
// robot ever stopped.
//
// A cell is damaged below about 3.00 V and the usual working floor is
// 3.3-3.5 V/cell, so these sit above that deliberately: a little runtime traded
// for pack life. Per cell they are the same numbers as before, just expressed
// in the unit that actually generalises.
#define BATT_WARN_V_PER_CELL     3.90f     // orange on the web UI, keep driving
#define BATT_SOFT_STOP_V_PER_CELL 3.80f    // stop driving, stay powered
#define BATT_CUTOFF_V_PER_CELL   3.733f     // latch STOP_BATTERY_LOW

// The shipped default. Change it from the web or the console rather than here -
// battery.cells is a parameter, so a robot that gets a different pack does not
// need reflashing.
#define BATT_CELLS              3
#define BATT_WARN_V             (BATT_CELLS * BATT_WARN_V_PER_CELL)
#define BATT_SOFT_STOP_V        (BATT_CELLS * BATT_SOFT_STOP_V_PER_CELL)
#define BATT_CUTOFF_V           (BATT_CELLS * BATT_CUTOFF_V_PER_CELL)
#define BATT_CUTOFF_HOLD_MS     3000      // must stay low this long (ignore load dips)

// Recovery from the SOFT stop, which is not latched and must not chatter.
//
// A pack at the soft-stop voltage sags under motor current and springs back the
// instant the motors stop. With no hysteresis the sequence is: sag, stop,
// recover, drive, sag - a start-stop oscillation that stresses the gearboxes
// and drains the pack it is meant to protect. Coming back out needs the voltage
// clearly above the threshold, and steady there.
#define BATT_SOFT_HYST_V        0.15f
#define BATT_SOFT_RECOVER_MS    3000

// There is deliberately NO unlatch voltage. The cutoff is cleared by switching
// the robot off and starting it again, and by nothing else. A pack sitting at
// the cutoff sags under load and springs back the moment the load comes off, so
// any recovery threshold just makes the robot stop, un-latch, drive, sag and
// stop again while it drains the cells past the point where they take damage.
// No voltage means "this pack is fine now" - only a person deciding to charge
// or swap it does. BATT_CUTOFF_CLEAR_V used to live here and was removed.

// Below this the motor rail is not powered (mushroom switch or relay open), so
// the pack voltage is unknown rather than low. Never latch on that.
#define PACK_RAIL_MIN_VALID_V   6.0f

// Sanity window for the rail that feeds the ESP32. Outside it the buck is
// misbehaving; reported on the web UI, never a reason to stop the robot alone.
//
// THIS IS 3V3 NOW, NOT 5 V. The ESP32 used to be fed from the 5 V rail through
// the devkit's on-board regulator, sharing that rail with the relay coil. It
// now has its own 3.3 V buck and VIN is left disconnected, which took the
// radio's transmit peaks off the coil's rail. Measured after the change: 3.42 V
// and 172 mA for the ESP32 and every sensor.
//
// The window was still 4.5-5.5 V, so a correctly wired board sat permanently
// outside it. Nothing consumed railOutOfRange() yet, which is the only reason
// it did not show up as a fault - the value was wrong and simply unread.
#define ESP32_RAIL_MIN_V        3.0f      // ESP32 needs 3.0 V to run its radio
#define ESP32_RAIL_MAX_V        3.6f      // absolute maximum for ESP32 and sensors

#endif // POWER_CONFIG_H
