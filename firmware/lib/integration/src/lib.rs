//! Spec-driven black-box integration tests for the OpenServoCore osc-native
//! stack (`docs/osc-native-protocol.md`). The [`sim`] module is a
//! discrete-event simulator with wire-event fidelity derived from the measured
//! silicon facts (sec 12): breaks, framing errors, byte ring timing, drive
//! discipline. Tests run the REAL `osc_servo_drivers::bus::ServoBus` and REAL
//! `osc_servo_core` dispatch against simulated providers -- the same code the chip
//! band will wire to hardware.

pub mod sim;
