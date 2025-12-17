//! Command execution - hardware-aware side effects.
//!
//! This module contains all the code that interacts with App and hardware.

use heapless::String;
use ufmt::uwrite;

use crate::fault::FaultKind;
use crate::safety::SafetyThresholds;
use crate::App;
use open_servo_control::ControlLoop;
use open_servo_hw::{BdcMotorDriver, DebugIo};
use open_servo_math::{CentiDeg, DeciC, DerivativeMode, Gain};
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

use super::command::{Command, FaultCmd, LimitCmd, PidCmd, PidField, SetCmd};
use super::DebugShell;

impl<D: DebugIo> DebugShell<D> {
    /// Execute a parsed command.
    pub(super) fn exec_command<C, H>(&mut self, app: &mut App<C>, hw: &mut H, cmd: Command)
    where
        C: ControlLoop,
        H: BdcMotorDriver,
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
        }
    }

    // ========================================================================
    // Command handlers
    // ========================================================================

    fn cmd_help(&mut self) {
        self.println("commands:");
        self.println("  help, ?                   - show this help");
        self.println("  state, s                  - show system state");
        self.println("  fault                     - show fault status");
        self.println("  fault clear               - clear latched fault");
        self.println("  set sp <cdeg>             - set setpoint");
        self.println("  pid pos                   - show PID config");
        self.println("  pid pos set kp <f32>      - set Kp gain");
        self.println("  pid pos set ki <f32>      - set Ki gain");
        self.println("  pid pos set kd <f32>      - set Kd gain");
        self.println("  pid pos set <kp> <ki> <kd> - set all gains");
        self.println("  pid pos mode err|meas     - set D mode");
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
            s.pwm_duty,
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
            let _ = uwrite!(buf, "T={}dC", temp.as_dc());
            self.println(&buf);
        }

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
            "current={}mA mcu_temp={}dC delta={}cdeg",
            t.current_limit.as_ma(),
            t.mcu_temp_limit.as_dc(),
            t.position_max_delta.as_cdeg(),
        );
        #[cfg(not(feature = "current-sense-bus"))]
        let _ = uwrite!(
            buf,
            "mcu_temp={}dC delta={}cdeg",
            t.mcu_temp_limit.as_dc(),
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
        if let Some(dc) = val {
            app.set_mcu_temp_limit(DeciC::from_dc(dc));
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "ok, mcu_temp={}dC", dc);
            self.println(&buf);
        } else {
            let t = app.get_thresholds();
            let mut buf: String<48> = String::new();
            let _ = uwrite!(buf, "mcu_temp={}dC", t.mcu_temp_limit.as_dc());
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
        app.set_thresholds(SafetyThresholds::default());
        self.println("ok, thresholds reset");
    }

    // ========================================================================
    // PID command handlers
    // ========================================================================

    fn cmd_pid_show<C: ControlLoop>(&mut self, app: &App<C>) {
        if let Some(cfg) = app.controller().pid_config() {
            let mut buf: String<96> = String::new();
            let mode_str = match cfg.derivative_mode {
                DerivativeMode::OnError => "err",
                DerivativeMode::OnMeasurement => "meas",
            };
            let _ = uwrite!(
                buf,
                "pos: kp={} ki={} kd={} mode={} out=[-{},{}]",
                cfg.kp, cfg.ki, cfg.kd, mode_str, cfg.output_max, cfg.output_max
            );
            self.println(&buf);
        } else {
            self.println("err: pid config not available");
        }
    }

    fn cmd_pid_set_one<C: ControlLoop>(&mut self, app: &mut App<C>, field: PidField, value: Gain) {
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

    fn cmd_pid_set_all<C: ControlLoop>(&mut self, app: &mut App<C>, kp: Gain, ki: Gain, kd: Gain) {
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

    fn cmd_pid_mode<C: ControlLoop>(&mut self, app: &mut App<C>, mode: DerivativeMode) {
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
}
