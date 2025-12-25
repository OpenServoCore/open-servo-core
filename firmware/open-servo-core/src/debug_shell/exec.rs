//! Command execution - hardware-aware side effects.
//!
//! This module contains all the code that interacts with App and hardware.

use heapless::String;
use ufmt::uwrite;

use crate::fault::FaultKind;
use crate::safety::SafetyThresholds;
use crate::App;
use open_servo_control::ControlLoop;
#[cfg(feature = "pid")]
use open_servo_control::PidTunable;
use open_servo_hw::{BdcMotorDriver, DebugIo};
use open_servo_hw::sensor::PositionSensor;
use open_servo_math::{CentiC, CentiDeg};
#[cfg(feature = "pid")]
use open_servo_math::{DerivativeMode, Gain};
#[cfg(feature = "current-sense-bus")]
use open_servo_math::MilliAmp;

/// Format FaultKind as a short string.
fn fault_str(kind: FaultKind) -> &'static str {
    match kind {
        FaultKind::OverCurrent => "overcurrent",
        FaultKind::McuOverTemp => "mcu_overtemp",
        FaultKind::MotorOverTemp => "motor_overtemp",
        FaultKind::DriverOverTemp => "driver_overtemp",
        FaultKind::UnderVoltage => "undervolt",
        FaultKind::QueuePressure => "queue",
        FaultKind::EncoderFault => "encoder",
        FaultKind::Stall => "stall",
        FaultKind::PositionError => "pos_error",
    }
}

use super::command::{Command, ComplianceCmd, FaultCmd, LimitCmd, MotorCmd, SetCmd};
#[cfg(feature = "pid")]
use super::command::{PidCmd, PidField};
use super::DebugShell;
use crate::servo_core::ServoMode;

impl<D: DebugIo> DebugShell<D> {
    /// Execute a parsed command.
    ///
    /// When the `pid` feature is enabled, this also requires `PidTunable`
    /// to support PID tuning commands.
    #[cfg(feature = "pid")]
    pub(super) fn exec_command<C, H>(&mut self, app: &mut App<C>, hw: &mut H, cmd: Command)
    where
        C: ControlLoop + PidTunable,
        H: BdcMotorDriver + PositionSensor,
    {
        self.exec_command_inner(app, hw, cmd)
    }

    /// Execute a parsed command (no PID tuning support).
    #[cfg(not(feature = "pid"))]
    pub(super) fn exec_command<C, H>(&mut self, app: &mut App<C>, hw: &mut H, cmd: Command)
    where
        C: ControlLoop,
        H: BdcMotorDriver + PositionSensor,
    {
        self.exec_command_inner(app, hw, cmd)
    }

    /// Inner command dispatcher.
    #[cfg(feature = "pid")]
    fn exec_command_inner<C, H>(&mut self, app: &mut App<C>, hw: &mut H, cmd: Command)
    where
        C: ControlLoop + PidTunable,
        H: BdcMotorDriver + PositionSensor,
    {
        match cmd {
            Command::Help => self.cmd_help(),
            Command::State => self.cmd_state(app),
            Command::Fault(FaultCmd::Show) => self.cmd_fault_show(app),
            Command::Fault(FaultCmd::Clear) => self.cmd_fault_clear(app, hw),
            Command::Set(SetCmd::Sp(val)) => self.cmd_set_sp(app, val),
            Command::Limit(LimitCmd::Show) => self.cmd_limit_show(app),
            #[cfg(feature = "current-sense-bus")]
            Command::Limit(LimitCmd::Current(val)) => self.cmd_limit_current(app, val),
            Command::Limit(LimitCmd::Temp(val)) => self.cmd_limit_temp(app, val),
            Command::Limit(LimitCmd::Delta(val)) => self.cmd_limit_delta(app, val),
            Command::Limit(LimitCmd::Faults(val)) => self.cmd_limit_faults(app, val),
            Command::Limit(LimitCmd::Pos(min, max)) => self.cmd_limit_pos(app, min, max),
            Command::Limit(LimitCmd::Stall(val)) => self.cmd_limit_stall(app, val),
            Command::Limit(LimitCmd::Error(val)) => self.cmd_limit_error(app, val),
            Command::Limit(LimitCmd::Reset) => self.cmd_limit_reset(app),
            Command::Pid(PidCmd::Show) => self.cmd_pid_show(app),
            Command::Pid(PidCmd::SetOne { field, value }) => {
                self.cmd_pid_set_one(app, field, value)
            }
            Command::Pid(PidCmd::SetAll { kp, ki, kd }) => self.cmd_pid_set_all(app, kp, ki, kd),
            Command::Pid(PidCmd::Mode(mode)) => self.cmd_pid_mode(app, mode),
            Command::Motor(MotorCmd::Status) => self.cmd_motor_status(app),
            Command::Motor(MotorCmd::Engage) => self.cmd_motor_engage(app, hw),
            Command::Motor(MotorCmd::Disengage) => self.cmd_motor_disengage(app, hw),
            Command::Compliance(ComplianceCmd::Show) => self.cmd_compliance_show(app),
            Command::Compliance(ComplianceCmd::MoveMa(ma)) => self.cmd_compliance_move_ma(app, ma),
            Command::Compliance(ComplianceCmd::HoldMa(ma)) => self.cmd_compliance_hold_ma(app, ma),
            Command::Compliance(ComplianceCmd::Vel(dps)) => self.cmd_compliance_vel(app, dps),
        }
    }

    /// Inner command dispatcher (no PID support).
    #[cfg(not(feature = "pid"))]
    fn exec_command_inner<C, H>(&mut self, app: &mut App<C>, hw: &mut H, cmd: Command)
    where
        C: ControlLoop,
        H: BdcMotorDriver + PositionSensor,
    {
        match cmd {
            Command::Help => self.cmd_help(),
            Command::State => self.cmd_state(app),
            Command::Fault(FaultCmd::Show) => self.cmd_fault_show(app),
            Command::Fault(FaultCmd::Clear) => self.cmd_fault_clear(app, hw),
            Command::Set(SetCmd::Sp(val)) => self.cmd_set_sp(app, val),
            Command::Limit(LimitCmd::Show) => self.cmd_limit_show(app),
            #[cfg(feature = "current-sense-bus")]
            Command::Limit(LimitCmd::Current(val)) => self.cmd_limit_current(app, val),
            Command::Limit(LimitCmd::Temp(val)) => self.cmd_limit_temp(app, val),
            Command::Limit(LimitCmd::Delta(val)) => self.cmd_limit_delta(app, val),
            Command::Limit(LimitCmd::Faults(val)) => self.cmd_limit_faults(app, val),
            Command::Limit(LimitCmd::Pos(min, max)) => self.cmd_limit_pos(app, min, max),
            Command::Limit(LimitCmd::Stall(val)) => self.cmd_limit_stall(app, val),
            Command::Limit(LimitCmd::Error(val)) => self.cmd_limit_error(app, val),
            Command::Limit(LimitCmd::Reset) => self.cmd_limit_reset(app),
            Command::Motor(MotorCmd::Status) => self.cmd_motor_status(app),
            Command::Motor(MotorCmd::Engage) => self.cmd_motor_engage(app, hw),
            Command::Motor(MotorCmd::Disengage) => self.cmd_motor_disengage(app, hw),
            Command::Compliance(ComplianceCmd::Show) => self.cmd_compliance_show(app),
            Command::Compliance(ComplianceCmd::MoveMa(ma)) => self.cmd_compliance_move_ma(app, ma),
            Command::Compliance(ComplianceCmd::HoldMa(ma)) => self.cmd_compliance_hold_ma(app, ma),
            Command::Compliance(ComplianceCmd::Vel(dps)) => self.cmd_compliance_vel(app, dps),
        }
    }

    // ========================================================================
    // Command handlers
    // ========================================================================

    fn cmd_help(&mut self) {
        self.println("commands:");
        self.println("  help, ?                   - show this help");
        self.println("  state, s                  - show system state");
        self.println("");
        self.println("  fault                     - show fault status");
        self.println("  fault clear               - clear latched fault");
        self.println("");
        self.println("  motor                     - show motor status");
        self.println("  motor engage              - engage motor (enable)");
        self.println("  motor disengage           - disengage motor (disable)");
        self.println("");
        self.println("  set sp <cdeg>             - set setpoint");
        self.println("");
        self.println("  pid pos                   - show PID config");
        self.println("  pid pos set kp <f32>      - set Kp gain");
        self.println("  pid pos set ki <f32>      - set Ki gain");
        self.println("  pid pos set kd <f32>      - set Kd gain");
        self.println("  pid pos set <kp> <ki> <kd> - set all gains");
        self.println("  pid pos mode err|meas     - set D mode");
        self.println("");
        self.println("  limit                     - show safety limits");
        #[cfg(feature = "current-sense-bus")]
        self.println("  limit current [mA]        - get/set current limit");
        self.println("  limit temp [dC]           - get/set temp limit");
        self.println("  limit delta [cdeg]        - get/set max pos delta");
        self.println("  limit faults [n]          - get/set fault count");
        self.println("  limit pos <min> <max>     - set position bounds");
        self.println("  limit stall [ticks]       - get/set stall timeout");
        self.println("  limit error [cdeg]        - get/set error limit");
        self.println("  limit reset               - restore defaults");
        self.println("");
        self.println("  compliance                - show compliance mode & settings");
        self.println("  compliance move <mA>      - set move mode current limit");
        self.println("  compliance hold <mA>      - set hold mode current limit");
        self.println("  compliance vel <dps>      - set backdrive velocity threshold");
    }

    fn cmd_state<C: ControlLoop>(&mut self, app: &App<C>) {
        let s = app.get_system_state();
        let mut buf: String<96> = String::new();

        // Position and setpoint (always available)
        let _ = uwrite!(
            buf,
            "sp={} pos={} pwm={}",
            s.setpoint.as_cdeg(),
            s.position.as_cdeg(),
            s.pwm_duty.as_raw(),
        );
        self.println(&buf);

        // Current (only with current-sense feature)
        buf.clear();
        #[cfg(feature = "current-sense-bus")]
        {
            if let Some(current) = s.current {
                let _ = uwrite!(buf, "I={}mA", current.as_ma());
            } else {
                let _ = uwrite!(buf, "I=n/a");
            }
        }

        // Voltage (optional)
        if let Some(voltage) = s.bus_voltage {
            let _ = uwrite!(buf, " V={}mV", voltage.as_mv());
        } else {
            let _ = uwrite!(buf, " V=n/a");
        }
        self.println(&buf);

        // Temperature (optional)
        if let Some(temp) = s.temperature {
            buf.clear();
            let _ = uwrite!(buf, "T={}cC", temp.as_centi_c());
            self.println(&buf);
        }
        
        // Motor temperature (thermal model)
        buf.clear();
        let motor_temp = app.get_motor_temp_deg();
        let motor_rise = app.get_motor_temp_rise_deg();
        let _ = uwrite!(buf, "Motor: {}°C (rise: {}°C)", motor_temp, motor_rise);
        self.println(&buf);

        let health = app.get_sensor_health();
        if health.bad_count() > 0 {
            buf.clear();
            let _ = uwrite!(buf, "sensor: {} bad reads", health.bad_count());
            self.println(&buf);
        }

        if let Some(kind) = app.fault_kind() {
            let mut buf: String<32> = String::new();
            let _ = uwrite!(buf, "FAULT: {}", fault_str(kind));
            self.println(&buf);
        } else {
            self.println("fault: none");
        }
    }

    fn cmd_fault_show<C: ControlLoop>(&mut self, app: &App<C>) {
        if let Some(kind) = app.fault_kind() {
            let mut buf: String<32> = String::new();
            let _ = uwrite!(buf, "FAULT: {}", fault_str(kind));
            self.println(&buf);
        } else {
            self.println("fault: none");
        }
    }

    fn cmd_fault_clear<C: ControlLoop, H: BdcMotorDriver>(&mut self, app: &mut App<C>, hw: &mut H) {
        app.clear_fault(hw);
        self.println("fault cleared");
    }

    fn cmd_set_sp<C: ControlLoop>(&mut self, app: &mut App<C>, val: i16) {
        let sp = CentiDeg::from_cdeg(val);
        app.set_setpoint(sp);
        let mut buf: String<64> = String::new();
        let _ = uwrite!(buf, "ok, sp={}", sp.as_cdeg());
        self.println(&buf);
    }

    fn cmd_limit_show<C: ControlLoop>(&mut self, app: &App<C>) {
        let t = app.get_thresholds();
        let mut buf: String<96> = String::new();

        #[cfg(feature = "current-sense-bus")]
        let _ = uwrite!(
            buf,
            "current={}mA mcu_temp={}cC delta={}cdeg",
            t.current_limit.as_ma(),
            t.mcu_temp_limit.as_centi_c(),
            t.position_max_delta.as_cdeg(),
        );
        #[cfg(not(feature = "current-sense-bus"))]
        let _ = uwrite!(
            buf,
            "mcu_temp={}cC delta={}cdeg",
            t.mcu_temp_limit.as_centi_c(),
            t.position_max_delta.as_cdeg(),
        );
        self.println(&buf);

        buf.clear();
        let _ = uwrite!(
            buf,
            "faults={} pos=[{},{}]cdeg",
            t.sensor_fault_count,
            t.position_min.as_cdeg(),
            t.position_max.as_cdeg(),
        );
        self.println(&buf);

        buf.clear();
        let _ = uwrite!(
            buf,
            "stall={}ticks error={}cdeg",
            t.stall_timeout_ticks,
            t.position_error_limit.as_cdeg(),
        );
        self.println(&buf);

        let health = app.get_sensor_health();
        buf.clear();
        let _ = uwrite!(buf, "bad_reads={}", health.bad_count());
        self.println(&buf);
    }

    #[cfg(feature = "current-sense-bus")]
    fn cmd_limit_current<C: ControlLoop>(&mut self, app: &mut App<C>, val: Option<i16>) {
        if let Some(ma) = val {
            app.set_current_limit(MilliAmp::from_ma(ma));
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "ok, current={}mA", ma);
            self.println(&buf);
        } else {
            let t = app.get_thresholds();
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "current={}mA", t.current_limit.as_ma());
            self.println(&buf);
        }
    }

    fn cmd_limit_temp<C: ControlLoop>(&mut self, app: &mut App<C>, val: Option<i16>) {
        if let Some(cc) = val {
            app.set_mcu_temp_limit(CentiC::from_centi_c(cc));
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "ok, mcu_temp={}cC", cc);
            self.println(&buf);
        } else {
            let t = app.get_thresholds();
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "mcu_temp={}cC", t.mcu_temp_limit.as_centi_c());
            self.println(&buf);
        }
    }

    fn cmd_limit_delta<C: ControlLoop>(&mut self, app: &mut App<C>, val: Option<i16>) {
        if let Some(cd) = val {
            app.set_position_max_delta(CentiDeg::from_cdeg(cd));
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "ok, delta={}cdeg", cd);
            self.println(&buf);
        } else {
            let t = app.get_thresholds();
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "delta={}cdeg", t.position_max_delta.as_cdeg());
            self.println(&buf);
        }
    }

    fn cmd_limit_faults<C: ControlLoop>(&mut self, app: &mut App<C>, val: Option<u8>) {
        if let Some(n) = val {
            app.set_sensor_fault_count(n);
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "ok, faults={}", n);
            self.println(&buf);
        } else {
            let t = app.get_thresholds();
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "faults={}", t.sensor_fault_count);
            self.println(&buf);
        }
    }

    fn cmd_limit_pos<C: ControlLoop>(&mut self, app: &mut App<C>, min: i16, max: i16) {
        app.set_position_bounds(CentiDeg::from_cdeg(min), CentiDeg::from_cdeg(max));
        let mut buf: String<48> = String::new();
        let _ = uwrite!(buf, "ok, pos=[{},{}]cdeg", min, max);
        self.println(&buf);
    }

    fn cmd_limit_stall<C: ControlLoop>(&mut self, app: &mut App<C>, val: Option<u16>) {
        if let Some(ticks) = val {
            app.set_stall_timeout(ticks);
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "ok, stall={}ticks", ticks);
            self.println(&buf);
        } else {
            let t = app.get_thresholds();
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "stall={}ticks", t.stall_timeout_ticks);
            self.println(&buf);
        }
    }

    fn cmd_limit_error<C: ControlLoop>(&mut self, app: &mut App<C>, val: Option<i16>) {
        if let Some(cdeg) = val {
            app.set_position_error_limit(CentiDeg::from_cdeg(cdeg));
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "ok, error={}cdeg", cdeg);
            self.println(&buf);
        } else {
            let t = app.get_thresholds();
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "error={}cdeg", t.position_error_limit.as_cdeg());
            self.println(&buf);
        }
    }

    fn cmd_limit_reset<C: ControlLoop>(&mut self, app: &mut App<C>) {
        // Note: These values should come from the board configuration
        // but we don't have access to it here. These match STM32F301 board.
        // TODO: Consider passing board config through App for runtime resets
        let thresholds = SafetyThresholds::new(
            1200,   // current_limit_ma
            8000,   // mcu_temp_limit_cc
            500,    // position_max_delta_cdeg
            10,     // sensor_fault_count
            0,      // position_min_cdeg
            18000,  // position_max_cdeg
            1000,   // stall_timeout_ticks
            10,     // stall_position_tolerance_cdeg
            3000,   // position_error_limit_cdeg
            50,     // position_error_timeout_ticks
        );
        app.set_thresholds(thresholds);
        self.println("ok, thresholds reset");
    }

    // ========================================================================
    // PID command handlers
    // ========================================================================

    #[cfg(feature = "pid")]
    fn cmd_pid_show<C: ControlLoop + PidTunable>(&mut self, app: &App<C>) {
        let cfg = app.controller().pid_config();
        let mut buf: String<96> = String::new();
        let mode_str = match cfg.derivative_mode {
            DerivativeMode::OnError => "err",
            DerivativeMode::OnMeasurement => "meas",
        };
        let _ = uwrite!(
            buf,
            "pos: kp={} ki={} kd={} mode={} out=[-32768,32767]",
            cfg.kp, cfg.ki, cfg.kd, mode_str
        );
        self.println(&buf);
    }

    #[cfg(feature = "pid")]
    fn cmd_pid_set_one<C: ControlLoop + PidTunable>(&mut self, app: &mut App<C>, field: PidField, value: Gain) {
        // Use the closure pattern to update config and rebuild atomically
        app.controller_mut().with_pid_config_mut(|cfg| match field {
            PidField::Kp => cfg.kp = value,
            PidField::Ki => cfg.ki = value,
            PidField::Kd => cfg.kd = value,
        });

        // Print confirmation
        let field_name = match field {
            PidField::Kp => "kp",
            PidField::Ki => "ki",
            PidField::Kd => "kd",
        };
        let mut buf: String<48> = String::new();
        let _ = uwrite!(buf, "ok, {}={}", field_name, value);
        self.println(&buf);
    }

    #[cfg(feature = "pid")]
    fn cmd_pid_set_all<C: ControlLoop + PidTunable>(&mut self, app: &mut App<C>, kp: Gain, ki: Gain, kd: Gain) {
        // Use the closure pattern to update all gains and rebuild atomically
        app.controller_mut().with_pid_config_mut(|cfg| {
            cfg.kp = kp;
            cfg.ki = ki;
            cfg.kd = kd;
        });

        let mut buf: String<64> = String::new();
        let _ = uwrite!(buf, "ok, kp={} ki={} kd={}", kp, ki, kd);
        self.println(&buf);
    }

    #[cfg(feature = "pid")]
    fn cmd_pid_mode<C: ControlLoop + PidTunable>(&mut self, app: &mut App<C>, mode: DerivativeMode) {
        // Use the closure pattern to update mode and rebuild atomically
        app.controller_mut().with_pid_config_mut(|cfg| {
            cfg.derivative_mode = mode;
        });

        let mode_str = match mode {
            DerivativeMode::OnError => "err",
            DerivativeMode::OnMeasurement => "meas",
        };
        let mut buf: String<32> = String::new();
        let _ = uwrite!(buf, "ok, mode={}", mode_str);
        self.println(&buf);
    }
    
    fn cmd_motor_status<C: ControlLoop>(&mut self, app: &App<C>) {
        let engaged = app.is_motor_engaged();
        let status = if engaged { "engaged" } else { "disengaged" };
        let mut buf: String<32> = String::new();
        let _ = uwrite!(buf, "motor: {}", status);
        self.println(&buf);
    }
    
    fn cmd_motor_engage<C: ControlLoop, H>(&mut self, app: &mut App<C>, hw: &mut H) 
    where
        H: BdcMotorDriver + open_servo_hw::sensor::PositionSensor,
    {
        app.engage_motor(hw);
        self.println("motor engaged");
    }
    
    fn cmd_motor_disengage<C: ControlLoop, H: BdcMotorDriver>(&mut self, app: &mut App<C>, hw: &mut H) {
        app.disengage_motor(hw);
        self.println("motor disengaged");
    }
    
    // ========================================================================
    // Compliance commands
    // ========================================================================
    
    fn cmd_compliance_show<C: ControlLoop>(&mut self, app: &App<C>) {
        let core = app.core();
        let mode = core.compliance_mode();
        let velocity = core.measured_velocity();
        
        let mode_str = match mode {
            ServoMode::Move => "MOVE",
            ServoMode::Hold => "HOLD",
            ServoMode::Yield => "YIELD",
        };
        
        let mut buf: String<64> = String::new();
        let _ = uwrite!(buf, "mode: {}, vel: {} dps", mode_str, velocity.as_dps10() / 10);
        self.println(&buf);
        
        // Show current limits
        buf.clear();
        let _ = uwrite!(buf, "move: 800mA, hold: 150mA"); // TODO: Make these accessible
        self.println(&buf);
    }
    
    fn cmd_compliance_move_ma<C: ControlLoop>(&mut self, app: &mut App<C>, ma: i16) {
        if ma < 100 || ma > 1500 {
            self.println("error: move limit must be 100-1500 mA");
            return;
        }
        
        app.core_mut().set_move_current_limit(ma);
        
        let mut buf: String<32> = String::new();
        let _ = uwrite!(buf, "move limit: {}mA", ma);
        self.println(&buf);
    }
    
    fn cmd_compliance_hold_ma<C: ControlLoop>(&mut self, app: &mut App<C>, ma: i16) {
        if ma < 100 || ma > 800 {
            self.println("error: hold limit must be 100-800 mA");
            return;
        }
        
        app.core_mut().set_hold_current_limit(ma);
        
        let mut buf: String<32> = String::new();
        let _ = uwrite!(buf, "hold limit: {}mA", ma);
        self.println(&buf);
    }
    
    fn cmd_compliance_vel<C: ControlLoop>(&mut self, _app: &mut App<C>, dps: i16) {
        if dps < 10 || dps > 100 {
            self.println("error: backdrive vel must be 10-100 deg/s");
            return;
        }
        
        // TODO: Make backdrive velocity threshold configurable
        let mut buf: String<48> = String::new();
        let _ = uwrite!(buf, "backdrive threshold: {} deg/s (not yet impl)", dps);
        self.println(&buf);
    }
}
