//! This module contains types etc that are copy+pasted from the firmware.

pub static mut CRC_LUT: [u8; 256] = [0; 256];
pub const CRC_POLY: u8 = 0xab;

pub const PARAMS_SIZE: usize = 76; // + message type, payload len, and crc.
pub const CONTROLS_SIZE: usize = 18; // + message type, payload len, and crc.

pub const MAX_PAYLOAD_SIZE: usize = PARAMS_SIZE; // For Params.
pub const MAX_PACKET_SIZE: usize = MAX_PAYLOAD_SIZE + 2; // + message type, payload len, and crc.

pub struct DecodeError {}

 // Time between querying the FC for readings, in ms.
pub const REFRESH_INTERVAL: u32 = 50;

use num_enum::TryFromPrimitive; // Enum from integer

use serde::Serialize;

// Note that serialize, and for ArmStatus, default, are not part of the firmware

#[derive(Clone, Copy, PartialEq, Serialize, TryFromPrimitive)]
#[repr(u8)]
/// For the switch position. We interpret actual mode from this, and other data, like prescense of GPS.
/// val is for passing over USB serial.
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
/// Repr is how this type is passed as serial.
pub enum MsgType {
    /// Transmit from FC
    Params = 0,
    SetMotorDirs = 1,
    /// Receive to FC
    ReqParams = 2,
    /// Acknowledgement, eg in response to setting something.
    Ack = 3,
    /// Controls data (From FC)
    Controls = 4,
    /// Request controls data. (From PC)
    ReqControls = 5,
}

impl MsgType {
    pub fn payload_size(&self) -> usize {
        match self {
            Self::Params => PARAMS_SIZE,
            Self::SetMotorDirs => 1, // Packed bits: motors 1-4, R-L. True = CW.
            Self::ReqParams => 0,
            Self::Ack => 0,
            Self::Controls => CONTROLS_SIZE,
            Self::ReqControls => 0,
        }
    }
}

pub struct Packet {
    message_type: MsgType,
    payload: [u8; MAX_PAYLOAD_SIZE], // todo?
    crc: u8,
}

/// Represents channel data in our end-use format.
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

/// Represents a first-order status of the drone. todo: What grid/reference are we using?
#[derive(Default, Serialize, Clone)]
pub struct Params {
    // todo: Do we want to use this full struct, or store multiple (3+) instantaneous ones?
    pub s_x: f32,
    pub s_y: f32,
    // Note that we only need to specify MSL vs AGL for position; velocity and accel should
    // be equiv for them.
    pub s_z_msl: f32,
    pub s_z_agl: f32,

    pub s_pitch: f32,
    pub s_roll: f32,
    pub s_yaw: f32,

    // Velocity
    pub v_x: f32,
    pub v_y: f32,
    pub v_z: f32,

    pub v_pitch: f32,
    pub v_roll: f32,
    pub v_yaw: f32,

    // Acceleration
    pub a_x: f32,
    pub a_y: f32,
    pub a_z: f32,

    pub a_pitch: f32,
    pub a_roll: f32,
    pub a_yaw: f32,
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
