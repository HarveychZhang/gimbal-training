#![no_std]
#![no_main]
#![forbid(unsafe_code)]

//! Gimbal debugging mode for the training rig.
//!
//! One control tick every 1 ms: read the remote and the motor feedback, choose the state,
//! compute the outputs, then send one 0x1FF voltage frame. `Waiting` and `Error` send zero.
//!
//! The rig geometry constants below (motor IDs, zero offsets, directions, travel limits and
//! the bench output limit) describe the actual bench hardware. The values here are starting
//! points; `NOTES.md` lists what must be measured and where to fill it in.

mod dm_mc02;

use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};
use embedded_can::Id;

// ---------------------------------------------------------------------------------------
// Rig configuration
// ---------------------------------------------------------------------------------------

/// GM6020 voltage group command. The rig was verified with 0x1FF; 0x1FE did not move it.
const MOTOR_COMMAND_ID: u16 = 0x1FF;
/// Motor ID from the DIP switch, and the matching feedback ID (0x205 + id - 1).
const YAW_MOTOR_ID: u8 = 1;
const PITCH_MOTOR_ID: u8 = 2;
const FEEDBACK_ID_BASE: u16 = 0x205;
const YAW_FEEDBACK_ID: u16 = FEEDBACK_ID_BASE + (YAW_MOTOR_ID as u16 - 1);
const PITCH_FEEDBACK_ID: u16 = FEEDBACK_ID_BASE + (PITCH_MOTOR_ID as u16 - 1);

/// Bench voltage limit in protocol counts; the protocol range is +/-25000.
/// First bench run: 2000. Raise in steps (4000, 6000) once direction and stopping
/// are confirmed; never exceed the protocol range.
const OUTPUT_LIMIT_RAW: f32 = 2_000.0;

/// Single-turn reading of the reference pose, in counts (0..8191), measured on the bench.
/// The joint angle is `direction * (raw * 2*PI / 8192 - zero)`, wrapped to +/-PI.
const YAW_ZERO_OFFSET_COUNTS: f32 = 0.0;
const PITCH_ZERO_OFFSET_COUNTS: f32 = 0.0;
/// +1 when a positive motor angle moves the joint in its positive direction.
const YAW_DIRECTION: f32 = 1.0;
const PITCH_DIRECTION: f32 = 1.0;

/// Pitch travel that the rig allows around the reference pose, in rad.
const PITCH_MIN_RAD: f32 = -0.6;
const PITCH_MAX_RAD: f32 = 0.6;

/// Full-stick target angular rate, in rad/s.
const YAW_TARGET_RATE_RAD_S: f32 = 2.0;
const PITCH_TARGET_RATE_RAD_S: f32 = 1.2;
/// Normalised stick magnitude below which the input counts as released.
const STICK_DEADBAND: f32 = 0.08;

/// Running limits: position loop output in rpm, speed loop output in counts.
const RUN_POSITION_MAX_RPM: f32 = 120.0;
const RUN_SPEED_MAX_RAW: f32 = OUTPUT_LIMIT_RAW;

/// Recentring limits used by `Prepare`.
const PREPARE_MAX_RPM: f32 = 80.0;
const PREPARE_MAX_RAW: f32 = 3_000.0;
/// Both axes must stay inside these error and speed limits for `PREPARE_SETTLE_TICKS`.
const PREPARE_ANGLE_TOL_RAD: f32 = 0.035;
const PREPARE_RPM_TOL: f32 = 25.0;
const PREPARE_SETTLE_TICKS: u16 = 200;
const PREPARE_TIMEOUT_TICKS: u16 = 4_000;

/// Feedback is fresh enough to enable after this many ticks; older than the second value
/// and a running state stops.
const FEEDBACK_FRESH_TICKS: u16 = 20;
const FEEDBACK_LOST_TICKS: u16 = 100;

/// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);
const DT_S: f32 = 0.001;

/// Stall: a real speed demand (`target_rpm` and output) with a position error that is not
/// closing and the shaft still slow. The angle-error condition keeps a normal start from
/// rest from looking like a stall.
/// The demand threshold sits below `PREPARE_MAX_RPM`, so recentring can detect it too.
const STALL_RPM_DEMAND: f32 = 50.0;
const STALL_RAW_DEMAND: f32 = 1_500.0;
const STALL_RPM_LIMIT: f32 = 25.0;
const STALL_ANGLE_ERR_RAD: f32 = 0.05;
const STALL_TICKS: u16 = 300;

/// Oscillation: the position error swings past +/-`OSC_AMP_RAD` and back, faster than
/// `OSC_FREQ_HZ`, for `OSC_TICKS` in a row.
const OSC_AMP_RAD: f32 = 0.03;
const OSC_FREQ_HZ: f32 = 25.0;
const OSC_TICKS: u16 = 300;
/// A half period shorter than this is treated as noise, not oscillation.
const OSC_MIN_HALF_PERIOD_TICKS: u32 = 5;
/// No crossing for this long and the estimate is discarded.
const OSC_STALE_TICKS: u32 = 200;
/// A crossing must be this recent for the oscillation to count as still running.
const OSC_ACTIVE_TICKS: u32 = 50;

/// Command frames dropped because the CAN transmit queue was full, for example while no
/// motor acknowledges frames.
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------------------

fn abs_f32(value: f32) -> f32 {
    if value < 0.0 { -value } else { value }
}

/// Wrap an angle into [-PI, PI], the shortest way round the shaft.
fn wrap_pi(mut angle: f32) -> f32 {
    while angle > core::f32::consts::PI {
        angle -= core::f32::consts::TAU;
    }
    while angle < -core::f32::consts::PI {
        angle += core::f32::consts::TAU;
    }
    angle
}

/// Normalise a stick channel (-660..=660) and remove the deadband, keeping full travel.
fn stick_input(channel: i16) -> f32 {
    let normalised = channel as f32 / 660.0;
    let magnitude = abs_f32(normalised);
    if magnitude <= STICK_DEADBAND {
        0.0
    } else {
        let scaled = (magnitude - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);
        if normalised < 0.0 { -scaled } else { scaled }
    }
}

/// Round half away from zero without `f32::round`, which needs `std`.
fn round_to_i16(value: f32) -> i16 {
    if value >= 0.0 {
        (value + 0.5) as i16
    } else {
        (value - 0.5) as i16
    }
}

// ---------------------------------------------------------------------------------------
// Motor feedback and commands
// ---------------------------------------------------------------------------------------

/// Feedback and voltage command for one motor.
#[derive(Default)]
pub struct Motor {
    /// Joint angle in rad relative to the calibrated reference pose, within +/-PI.
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// Feedback current as sent on the wire. The motor manual gives no scale to amperes.
    pub current_raw: i16,
    /// Voltage command counts, not volts; protocol range +/-25000.
    pub command_voltage_raw: f32,
    /// Reference-pose reading in rad and the installed direction of this joint.
    zero_offset_rad: f32,
    direction: f32,
}

impl Motor {
    /// Clear the voltage command in RAM. The motor receives zero when
    /// `Control::transmit` next runs, within 1 ms.
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }

    /// Decode one 8-byte GM6020 feedback frame: 0..1 angle, 2..3 rpm, 4..5 current,
    /// 6 temperature (not used here), 7 reserved.
    pub fn read(&mut self, data: &[u8; 8]) {
        let raw_angle = u16::from_be_bytes([data[0], data[1]]) & 0x1FFF;
        let mechanical_rad = raw_angle as f32 * (core::f32::consts::TAU / 8192.0);
        self.angle_rad = wrap_pi(self.direction * (mechanical_rad - self.zero_offset_rad));
        self.speed_rpm = self.direction * i16::from_be_bytes([data[2], data[3]]) as f32;
        self.current_raw = i16::from_be_bytes([data[4], data[5]]);
    }
}

// ---------------------------------------------------------------------------------------
// PID
// ---------------------------------------------------------------------------------------

/// PID gains, plus the integral and history kept between ticks.
///
/// `calculate` differentiates the measurement rather than the error, so a step in the
/// target does not kick the output through Kd. The output is limited to
/// `[-maxout, maxout]`; with `maxout = 0` the loop returns zero, which is how the position
/// loop is switched off while tuning the speed loop.
#[derive(Default)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Limit of the integral term, in output units; 0 disables integration.
    pub integral_limit: f32,
    /// Symmetric output limit, in output units.
    pub maxout: f32,
    integral: f32,
    prev_measured: f32,
    has_prev: bool,
}

impl Pid {
    /// Clear the integral and history.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measured = 0.0;
        self.has_prev = false;
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns voltage counts. `dt_s` is in
    /// seconds. The result is finite and within +/-`maxout`.
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        let dt = if dt_s > 0.0 { dt_s } else { DT_S };
        let error = target - measured;

        let proportional = self.kp * error;

        if self.ki != 0.0 && self.maxout > 0.0 {
            self.integral += self.ki * error * dt;
            self.integral = self
                .integral
                .clamp(-self.integral_limit, self.integral_limit);
        }

        let derivative = if self.has_prev {
            -self.kd * (measured - self.prev_measured) / dt
        } else {
            0.0
        };
        self.prev_measured = measured;
        self.has_prev = true;

        let output = proportional + self.integral + derivative;
        let limit = if self.maxout > 0.0 { self.maxout } else { 0.0 };
        let limited = output.clamp(-limit, limit);
        if limited.is_finite() { limited } else { 0.0 }
    }
}

// ---------------------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------------------

/// The gimbal's stage, which decides the allowed operations and outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Zero output. Waits for valid remote input and fresh feedback from both motors,
    /// then for a new enable request.
    Waiting,
    /// Recenters both axes to the reference pose with limited speed and output.
    /// Enters `Running` once both axes settle.
    Prepare,
    /// The left stick moves each axis's `target_angle_rad`, and its cascade tracks it.
    Running,
    /// Zero output until a manual recovery.
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxisId {
    Yaw,
    Pitch,
    /// Used when a fault is not tied to one axis, such as a recentring timeout.
    Both,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultKind {
    /// A speed demand with a small measured speed for `STALL_TICKS`.
    Stall,
    /// Sustained oscillation above `OSC_FREQ_HZ`.
    Oscillation,
    /// `Prepare` did not settle within `PREPARE_TIMEOUT_TICKS`.
    PrepareTimeout,
}

#[derive(Clone, Copy, Debug)]
pub struct Fault {
    pub axis: AxisId,
    pub kind: FaultKind,
}

/// Counts position-error zero crossings and turns alternating crossings into a frequency.
/// Two opposite crossings are one full round trip, so `frequency = 1 / (2 * half_period)`.
#[derive(Default)]
struct OscDetector {
    last_sign: i8,
    last_cross_tick: u32,
    freq_hz: f32,
    above_ticks: u16,
}

impl OscDetector {
    fn reset(&mut self) {
        *self = Self::default();
    }

    /// Returns true when the oscillation has stayed above the frequency limit long enough.
    fn update(&mut self, tick: u32, error_rad: f32) -> bool {
        if self.last_cross_tick != 0 && tick.wrapping_sub(self.last_cross_tick) > OSC_STALE_TICKS {
            self.freq_hz = 0.0;
            self.above_ticks = 0;
            self.last_sign = 0;
        }

        let new_sign = if error_rad > OSC_AMP_RAD {
            1
        } else if error_rad < -OSC_AMP_RAD {
            -1
        } else {
            self.last_sign
        };

        if new_sign != 0 && new_sign != self.last_sign && self.last_sign != 0 {
            let half_period = tick.wrapping_sub(self.last_cross_tick);
            if half_period >= OSC_MIN_HALF_PERIOD_TICKS {
                self.freq_hz = 1000.0 / (2.0 * half_period as f32);
            }
            self.last_cross_tick = tick;
        } else if self.last_cross_tick == 0 && new_sign != 0 {
            self.last_cross_tick = tick;
        }
        self.last_sign = new_sign;

        let recent_crossing = self.last_cross_tick != 0
            && tick.wrapping_sub(self.last_cross_tick) <= OSC_ACTIVE_TICKS;
        if self.freq_hz > OSC_FREQ_HZ && recent_crossing {
            self.above_ticks = self.above_ticks.saturating_add(1);
        } else {
            self.above_ticks = 0;
        }
        self.above_ticks >= OSC_TICKS
    }
}

/// One gimbal axis: its motor, targets and cascaded PIDs.
#[derive(Default)]
pub struct Axis {
    pub motor: Motor,
    /// Position loop target in rad.
    pub target_angle_rad: f32,
    /// Speed loop target in rpm, set from the position loop output.
    pub target_rpm: f32,
    /// Position loop: rad in, rpm out.
    pub position_pid: Pid,
    /// Speed loop: rpm in, voltage counts out.
    pub speed_pid: Pid,
}

/// State shared by every control tick.
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,
    /// Ticks since the last valid feedback frame per axis; `u16::MAX` means none yet.
    pub yaw_feedback_age: u16,
    pub pitch_feedback_age: u16,
    /// Set by a down position of the right switch; a middle position then starts the rig.
    /// Cleared on every enable so the rig never re-enables from a held switch.
    pub armed: bool,
    /// Last fault, kept for display until a manual recovery.
    pub fault: Option<Fault>,
    last_left_switch: u8,
    prepare_ticks: u16,
    prepare_settle_ticks: u16,
    stall_ticks: [u16; 2],
    osc: [OscDetector; 2],
    tick: u32,
}

/// Start in `Waiting` with no remote input.
/// Set the initial value of each field you add here, including the PID gains.
impl Default for Control {
    fn default() -> Self {
        let mut yaw = Axis::default();
        yaw.motor.zero_offset_rad = YAW_ZERO_OFFSET_COUNTS * (core::f32::consts::TAU / 8192.0);
        yaw.motor.direction = YAW_DIRECTION;
        // Position: rad -> rpm; 150 rpm per rad, 120 rpm of travel for the speed loop.
        // The small integral removes the droop that gravity and friction would otherwise
        // leave behind; set it to 0 when starting the position-loop tuning from scratch.
        yaw.position_pid = Pid {
            kp: 150.0,
            ki: 20.0,
            integral_limit: 60.0,
            maxout: RUN_POSITION_MAX_RPM,
            ..Pid::default()
        };
        // Speed: rpm -> voltage counts; 80 counts per rpm, integral limited to 3000 counts.
        yaw.speed_pid = Pid {
            kp: 80.0,
            integral_limit: 3_000.0,
            maxout: OUTPUT_LIMIT_RAW,
            ..Pid::default()
        };

        let mut pitch = Axis::default();
        pitch.motor.zero_offset_rad = PITCH_ZERO_OFFSET_COUNTS * (core::f32::consts::TAU / 8192.0);
        pitch.motor.direction = PITCH_DIRECTION;
        pitch.position_pid = Pid {
            kp: 150.0,
            ki: 20.0,
            integral_limit: 60.0,
            maxout: RUN_POSITION_MAX_RPM,
            ..Pid::default()
        };
        pitch.speed_pid = Pid {
            kp: 80.0,
            integral_limit: 3_000.0,
            maxout: OUTPUT_LIMIT_RAW,
            ..Pid::default()
        };

        Self {
            state: State::Waiting,
            yaw,
            pitch,
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            yaw_feedback_age: u16::MAX,
            pitch_feedback_age: u16::MAX,
            armed: false,
            fault: None,
            last_left_switch: 3,
            prepare_ticks: 0,
            prepare_settle_ticks: 0,
            stall_ticks: [0; 2],
            osc: [OscDetector::default(), OscDetector::default()],
            tick: 0,
        }
    }
}

impl Control {
    /// Clear both motors' voltage commands in RAM.
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    fn remote_switch(&self) -> (u8, u8) {
        match self.remote {
            Some(remote) => (remote.right_switch, remote.left_switch),
            None => (3, 3),
        }
    }

    fn feedback_fresh(&self) -> bool {
        self.yaw_feedback_age < FEEDBACK_FRESH_TICKS
            && self.pitch_feedback_age < FEEDBACK_FRESH_TICKS
    }

    fn feedback_ok(&self) -> bool {
        self.yaw_feedback_age < FEEDBACK_LOST_TICKS && self.pitch_feedback_age < FEEDBACK_LOST_TICKS
    }

    /// Runs every 1 ms: read the remote and motor feedback, update the state, then
    /// compute and send the outputs. Missing remote input stops output; a latched Error is preserved.
    pub fn tick(&mut self, can: &mut Can1) {
        self.tick = self.tick.wrapping_add(1);
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();
        self.update();
        self.transmit(can);
    }

    /// Take the latest input published by the remote task. After an invalid frame, or
    /// `REMOTE_SILENT_TICKS` ticks without one, clear the commands; return to Waiting unless already in Error.
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            let was_disconnected = self.remote.is_none();
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
                if was_disconnected {
                    // Do not let a switch that moved while the link was down count as a
                    // gesture: the next enable or recovery needs a fresh edge.
                    let (_, left) = self.remote_switch();
                    self.last_left_switch = left;
                    self.armed = false;
                }
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.disable_all();
            if self.state != State::Error {
                if self.state != State::Waiting {
                    rtt_target::rprintln!("Remote lost: output off, back to Waiting");
                }
                self.state = State::Waiting;
            }
            self.armed = false;
            self.prepare_ticks = 0;
            self.prepare_settle_ticks = 0;
        }
    }

    /// Read every queued CAN frame without waiting for more.
    pub fn read_feedback(&mut self, can: &mut Can1) {
        self.yaw_feedback_age = self.yaw_feedback_age.saturating_add(1);
        self.pitch_feedback_age = self.pitch_feedback_age.saturating_add(1);

        let receiver = can.reader();
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    let frame = envelope.frame;
                    let id = match frame.id() {
                        Id::Standard(id) => id.as_raw(),
                        Id::Extended(_) => continue,
                    };
                    if frame.data().len() != 8 {
                        continue;
                    }
                    let Ok(data) = <&[u8; 8]>::try_from(frame.data()) else {
                        continue;
                    };
                    match id {
                        YAW_FEEDBACK_ID => {
                            self.yaw.motor.read(data);
                            self.yaw_feedback_age = 0;
                        }
                        PITCH_FEEDBACK_ID => {
                            self.pitch.motor.read(data);
                            self.pitch_feedback_age = 0;
                        }
                        _ => {}
                    }
                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    /// Choose the state from the remote, feedback and fault conditions.
    pub fn update_mode(&mut self) {
        let (right_switch, left_switch) = self.remote_switch();

        if self.state != State::Error && right_switch == 2 {
            self.armed = true;
        }

        match self.state {
            State::Waiting => {
                if self.remote.is_some() && self.feedback_fresh() && self.armed && right_switch == 3
                {
                    self.enter_prepare();
                }
            }
            State::Prepare => {
                if right_switch == 2 {
                    rtt_target::rprintln!("Disable in Prepare: output off");
                    self.stop_to_waiting(true);
                } else if !self.feedback_ok() {
                    rtt_target::rprintln!("Feedback lost in Prepare: output off");
                    self.stop_to_waiting(false);
                } else {
                    self.prepare_ticks = self.prepare_ticks.saturating_add(1);
                    if self.prepare_settled() {
                        self.prepare_settle_ticks = self.prepare_settle_ticks.saturating_add(1);
                    } else {
                        self.prepare_settle_ticks = 0;
                    }

                    if self.prepare_settle_ticks >= PREPARE_SETTLE_TICKS {
                        self.state = State::Running;
                        self.prepare_settle_ticks = 0;
                        // Restore the debugging limits after the slower recentring.
                        self.yaw.position_pid.maxout = RUN_POSITION_MAX_RPM;
                        self.pitch.position_pid.maxout = RUN_POSITION_MAX_RPM;
                        self.yaw.speed_pid.maxout = RUN_SPEED_MAX_RAW;
                        self.pitch.speed_pid.maxout = RUN_SPEED_MAX_RAW;
                        self.stall_ticks = [0; 2];
                        self.osc[0].reset();
                        self.osc[1].reset();
                        rtt_target::rprintln!(
                            "Prepare done: Running at yaw={:.3} pitch={:.3}",
                            self.yaw.motor.angle_rad,
                            self.pitch.motor.angle_rad
                        );
                    } else if self.prepare_ticks >= PREPARE_TIMEOUT_TICKS {
                        self.fail(Fault {
                            axis: AxisId::Both,
                            kind: FaultKind::PrepareTimeout,
                        });
                    } else {
                        self.check_faults();
                    }
                }
            }
            State::Running => {
                if right_switch == 2 {
                    rtt_target::rprintln!("Disable: output off");
                    self.stop_to_waiting(true);
                } else if !self.feedback_ok() {
                    rtt_target::rprintln!("Feedback lost: output off");
                    self.stop_to_waiting(false);
                } else {
                    self.check_faults();
                }
            }
            State::Error => {
                // Manual recovery: right switch holds the middle position while the left
                // switch goes down and then up. A held switch never recovers by itself.
                if self.feedback_fresh()
                    && right_switch == 3
                    && self.last_left_switch == 2
                    && left_switch == 1
                {
                    self.recover();
                }
            }
        }

        self.last_left_switch = left_switch;
    }

    /// Set targets for the current state, run each axis's PIDs and write the voltage commands.
    pub fn update(&mut self) {
        match self.state {
            State::Waiting | State::Error => {
                self.disable_all();
                self.yaw.target_rpm = 0.0;
                self.pitch.target_rpm = 0.0;
            }
            State::Prepare => {
                // Targets stay at the reference pose for the whole recentring.
                self.cascade(DT_S);
            }
            State::Running => {
                self.move_targets_from_stick();
                self.cascade(DT_S);
            }
        }
    }

    /// Queue one group command frame on CAN1 without waiting. This runs every tick, so a
    /// cleared command reaches the motor within 1 ms. A full queue drops the frame and
    /// counts it in `TX_FAILED`.
    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data = [0u8; 8];
        for (motor_id, motor) in [
            (YAW_MOTOR_ID, &self.yaw.motor),
            (PITCH_MOTOR_ID, &self.pitch.motor),
        ] {
            let slot = (usize::from(motor_id) - 1) * 2;
            let counts = round_to_i16(
                motor
                    .command_voltage_raw
                    .clamp(-OUTPUT_LIMIT_RAW, OUTPUT_LIMIT_RAW),
            );
            data[slot..slot + 2].copy_from_slice(&counts.to_be_bytes());
        }
        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }

    // -- state helpers ------------------------------------------------------------------

    fn enter_prepare(&mut self) {
        self.state = State::Prepare;
        self.prepare_ticks = 0;
        self.prepare_settle_ticks = 0;
        self.yaw.target_angle_rad = 0.0;
        self.pitch.target_angle_rad = 0.0;
        self.yaw.target_rpm = 0.0;
        self.pitch.target_rpm = 0.0;
        self.yaw.position_pid.reset();
        self.yaw.speed_pid.reset();
        self.pitch.position_pid.reset();
        self.pitch.speed_pid.reset();
        // Recentring runs slower than remote debugging.
        self.yaw.position_pid.maxout = PREPARE_MAX_RPM;
        self.pitch.position_pid.maxout = PREPARE_MAX_RPM;
        self.yaw.speed_pid.maxout = PREPARE_MAX_RAW;
        self.pitch.speed_pid.maxout = PREPARE_MAX_RAW;
        self.armed = false;
        self.fault = None;
        self.stall_ticks = [0; 2];
        self.osc[0].reset();
        self.osc[1].reset();
        rtt_target::rprintln!(
            "Enable: Prepare, yaw={:.3} pitch={:.3}",
            self.yaw.motor.angle_rad,
            self.pitch.motor.angle_rad
        );
    }

    fn stop_to_waiting(&mut self, keep_armed: bool) {
        self.state = State::Waiting;
        self.disable_all();
        self.yaw.target_rpm = 0.0;
        self.pitch.target_rpm = 0.0;
        self.prepare_ticks = 0;
        self.prepare_settle_ticks = 0;
        self.armed = keep_armed;
        self.fault = None;
    }

    /// True when both axes are close to the reference pose and nearly still.
    fn prepare_settled(&self) -> bool {
        let yaw_error = wrap_pi(self.yaw.target_angle_rad - self.yaw.motor.angle_rad);
        let pitch_error = wrap_pi(self.pitch.target_angle_rad - self.pitch.motor.angle_rad);
        abs_f32(yaw_error) <= PREPARE_ANGLE_TOL_RAD
            && abs_f32(pitch_error) <= PREPARE_ANGLE_TOL_RAD
            && abs_f32(self.yaw.motor.speed_rpm) <= PREPARE_RPM_TOL
            && abs_f32(self.pitch.motor.speed_rpm) <= PREPARE_RPM_TOL
    }

    /// Move each target angle by the left stick. Left-right moves yaw, up-down pitch; the
    /// target holds its value when the stick returns to the deadband.
    fn move_targets_from_stick(&mut self) {
        let Some(remote) = self.remote else { return };
        let yaw_input = stick_input(remote.channels[2]);
        let pitch_input = stick_input(remote.channels[3]);

        self.yaw.target_angle_rad =
            wrap_pi(self.yaw.target_angle_rad + yaw_input * YAW_TARGET_RATE_RAD_S * DT_S);
        self.pitch.target_angle_rad = (self.pitch.target_angle_rad
            + pitch_input * PITCH_TARGET_RATE_RAD_S * DT_S)
            .clamp(PITCH_MIN_RAD, PITCH_MAX_RAD);
    }

    /// Run both cascades with the limits currently set on the PIDs.
    fn cascade(&mut self, dt_s: f32) {
        cascade_axis(&mut self.yaw, dt_s);
        cascade_axis(&mut self.pitch, dt_s);
    }

    /// Detect stall and sustained oscillation on both axes, and latch `Error` on the first.
    fn check_faults(&mut self) {
        let mut detected = None;
        for index in 0..2 {
            let (id, axis) = if index == 0 {
                (AxisId::Yaw, &self.yaw)
            } else {
                (AxisId::Pitch, &self.pitch)
            };
            let error = wrap_pi(axis.target_angle_rad - axis.motor.angle_rad);
            let speed = axis.motor.speed_rpm;
            let demand = abs_f32(axis.target_rpm) >= STALL_RPM_DEMAND
                && abs_f32(axis.motor.command_voltage_raw) >= STALL_RAW_DEMAND
                && abs_f32(error) >= STALL_ANGLE_ERR_RAD;

            if demand && abs_f32(speed) <= STALL_RPM_LIMIT {
                self.stall_ticks[index] = self.stall_ticks[index].saturating_add(1);
            } else {
                self.stall_ticks[index] = 0;
            }

            let oscillating = self.osc[index].update(self.tick, error);

            if self.stall_ticks[index] >= STALL_TICKS {
                detected = Some(Fault {
                    axis: id,
                    kind: FaultKind::Stall,
                });
            } else if oscillating {
                detected = Some(Fault {
                    axis: id,
                    kind: FaultKind::Oscillation,
                });
            }
            if detected.is_some() {
                break;
            }
        }
        if let Some(fault) = detected {
            self.fail(fault);
        }
    }

    fn fail(&mut self, fault: Fault) {
        self.state = State::Error;
        self.disable_all();
        self.fault = Some(fault);
        self.armed = false;
        self.prepare_ticks = 0;
        self.prepare_settle_ticks = 0;
        rtt_target::rprintln!(
            "ERROR: {:?} on {:?}; output off, yaw={:.3} pitch={:.3}",
            fault.kind,
            fault.axis,
            self.yaw.motor.angle_rad,
            self.pitch.motor.angle_rad
        );
    }

    /// Leave `Error` from the current pose: hold both angles where they are, clear the PID
    /// history and the detection counters, and resume remote debugging without recentring.
    fn recover(&mut self) {
        self.state = State::Running;
        self.yaw.target_angle_rad = self.yaw.motor.angle_rad;
        self.pitch.target_angle_rad = self.pitch.motor.angle_rad;
        self.yaw.target_rpm = 0.0;
        self.pitch.target_rpm = 0.0;
        self.yaw.position_pid.reset();
        self.yaw.speed_pid.reset();
        self.pitch.position_pid.reset();
        self.pitch.speed_pid.reset();
        self.yaw.position_pid.maxout = RUN_POSITION_MAX_RPM;
        self.pitch.position_pid.maxout = RUN_POSITION_MAX_RPM;
        self.yaw.speed_pid.maxout = RUN_SPEED_MAX_RAW;
        self.pitch.speed_pid.maxout = RUN_SPEED_MAX_RAW;
        self.stall_ticks = [0; 2];
        self.osc[0].reset();
        self.osc[1].reset();
        self.fault = None;
        self.prepare_ticks = 0;
        self.prepare_settle_ticks = 0;
        rtt_target::rprintln!(
            "Recovered: Running at yaw={:.3} target={:.3} pitch={:.3} target={:.3}",
            self.yaw.motor.angle_rad,
            self.yaw.target_angle_rad,
            self.pitch.motor.angle_rad,
            self.pitch.target_angle_rad
        );
    }
}

/// Position loop -> speed loop -> voltage command for one axis.
fn cascade_axis(axis: &mut Axis, dt_s: f32) {
    let rpm_target = axis
        .position_pid
        .calculate(axis.target_angle_rad, axis.motor.angle_rad, dt_s);
    let limit = axis.position_pid.maxout.max(0.0);
    axis.target_rpm = rpm_target.clamp(-limit, limit);
    axis.motor.command_voltage_raw =
        axis.speed_pid
            .calculate(axis.target_rpm, axis.motor.speed_rpm, dt_s);
}

// --------------------------------------------------------------------------------------
// Entry point and task
// --------------------------------------------------------------------------------------

// Set up RTT and the board, then start the remote and control tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("Gimbal control started");
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// Run `tick` at 1 kHz and feed the watchdog after each one. If a tick hangs, the watchdog
// resets the MCU after 100 ms.
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    let mut log_ticks = 0u16;
    loop {
        ticker.next().await;
        // After a stall, such as the 20 ms halt when a debug probe attaches, start the
        // schedule again instead of running every missed tick back to back.
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        control.tick(&mut can);
        watchdog.pet();

        log_ticks += 1;
        if log_ticks >= 200 {
            log_ticks = 0;
            rtt_target::rprintln!(
                "state={:?} yaw a={:.3} t={:.3} rpm={:.1} trpm={:.1} out={:.0} | pitch a={:.3} t={:.3} rpm={:.1} trpm={:.1} out={:.0}",
                control.state,
                control.yaw.motor.angle_rad,
                control.yaw.target_angle_rad,
                control.yaw.motor.speed_rpm,
                control.yaw.target_rpm,
                control.yaw.motor.command_voltage_raw,
                control.pitch.motor.angle_rad,
                control.pitch.target_angle_rad,
                control.pitch.motor.speed_rpm,
                control.pitch.target_rpm,
                control.pitch.motor.command_voltage_raw
            );
        }
    }
}
