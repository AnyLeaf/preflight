//! This module contains types etc that are copy+pasted from the firmware.

pub static mut CRC_LUT: [u8; 256] = [0; 256];
pub const CRC_POLY: u8 = 0xab;

const QUATERNION_SIZE: usize = F32_BYTES * 4;
const PARAMS_SIZE: usize = QUATERNION_SIZE + 4 * 3;
const CONTROLS_SIZE: usize = 18;

const PARAMS_PACKET_SIZE: usize = PARAMS_SIZE + 2;
const CONTROLS_PACKET_SIZE: usize = CONTROLS_SIZE + 2;

pub struct DecodeError {}

 // Time between querying the FC for readings, in ms.
pub const REFRESH_INTERVAL: u32 = 50;

use num_enum::TryFromPrimitive; // Enum from integer

use serde::Serialize;

// Note that serialize, and for ArmStatus, default, are not part of the firmware

#[derive(Clone, Copy, PartialEq, Serialize, TryFromPrimitive)]
#[repr(u8)]
pub enum InputModeSwitch {
    /// Acro mode
    Acro = 0,
    /// Command if GPS is present; Attitude if not
    AttitudeCommand = 1,
}

impl Default for InputModeSwitch {
    fn default() -> Self {
        Self::Acro
    }
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Serialize, TryFromPrimitive)]
pub enum ArmStatus {
    /// Motors are [pre]disarmed
    Disarmed = 0,
    /// Motors are [pre]armed
    Armed = 1,
}

impl Default for ArmStatus {
    fn default() -> Self {
        Self::Disarmed
    }
}

#[derive(Clone, Copy, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum MsgType {
    Params = 0,
    SetMotorDirs = 1,
    ReqParams = 2,
    Ack = 3,
    Controls = 4,
    ReqControls = 5,
    ArmMotors = 6,
    DisarmMotors = 7,
    /// Start a specific motor
    StartMotor = 8,
    /// Stop a specific motor
    StopMotor = 9,
}

impl MsgType {
    pub fn payload_size(&self) -> usize {
        match self {
            // Self::Params => PARAMS_SIZE,
            Self::Params => ATTITUDE_SIZE,
            Self::SetMotorDirs => 1, // Packed bits: motors 1-4, R-L. True = CW.
            Self::ReqParams => 0,
            Self::Ack => 0,
            Self::Controls => CONTROLS_SIZE,
            Self::ReqControls => 0,
            Self::ArmMotors => 0,
            Self::DisarmMotors => 0,
            Self::StartMotor => 1,
            Self::StopMotor => 1,
        }
    }
}

#[derive(Default, Serialize, Clone)]
pub struct ChannelData {
    /// Aileron, -1. to 1.
    pub roll: f32,
    /// Elevator, -1. to 1.
    pub pitch: f32,
    /// Throttle, 0. to 1., or -1. to 1. depending on if stick auto-centers.
    pub throttle: f32,
    /// Rudder, -1. to 1.
    pub yaw: f32,
    pub arm_status: ArmStatus,
    pub input_mode: InputModeSwitch,
    // pub alt_hold: AltHoldSwitch, // todo
    // todo: Auto-recover commanded, auto-TO/land/RTB, obstacle avoidance etc.
}

// #[derive(Default, Serialize, Clone)]
// pub struct Params {
//     // todo: Do we want to use this full struct, or store multiple (3+) instantaneous ones?
//     pub s_x: f32,
//     pub s_y: f32,
//     // Note that we only need to specify MSL vs AGL for position; velocity and accel should
//     // be equiv for them.
//     pub s_z_msl: f32,
//     pub s_z_agl: f32,
//
//     pub s_pitch: f32,
//     pub s_roll: f32,
//     pub s_yaw: f32,
//
//     pub quaternion: Quaternion,
//
//     // Velocity
//     pub v_x: f32,
//     pub v_y: f32,
//     pub v_z: f32,
//
//     pub v_pitch: f32,
//     pub v_roll: f32,
//     pub v_yaw: f32,
//
//     // Acceleration
//     pub a_x: f32,
//     pub a_y: f32,
//     pub a_z: f32,
//
//     pub a_pitch: f32,
//     pub a_roll: f32,
//     pub a_yaw: f32,
// }

#[derive(Clone, Copy)]
pub enum Rotor {
    R1,
    R2,
    R3,
    R4,
}

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum RotorPosition {
    FrontLeft = 0,
    FrontRight = 1,
    AftLeft = 2,
    AftRight = 3,
}

pub fn crc_init(lut: &mut [u8; 256], poly: u8) {
    for i in 0..256 {
        let mut crc = i as u8;
        for _ in 0..8 {
            crc = (crc << 1) ^ (if (crc & 0x80) > 0 { poly } else { 0 });
        }
        lut[i] = crc & 0xff;
    }
}

pub fn calc_crc(lut: &[u8; 256], data: &[u8], mut size: u8) -> u8 {
    let mut crc = 0;
    let mut i = 0;

    while size > 0 {
        size -= 1;
        crc = lut[(crc ^ data[i]) as usize];
        i += 1;
    }
    crc
}

#[derive(Clone, Copy, Default, Serialize)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
