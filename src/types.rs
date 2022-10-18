//! This module contains types etc that are copy+pasted from the firmware.

pub const F32_BYTES: usize = 4;

const CRC_POLY: u8 = 0xab;
const CRC_LUT: [u8; 256] = crc_init(CRC_POLY);

pub const QUATERNION_SIZE: usize = F32_BYTES * 4; // Quaternion (4x4 + altimeter + voltage reading + current reading)
pub const PARAMS_SIZE: usize = QUATERNION_SIZE + F32_BYTES * 4 + 1; //
pub const CONTROLS_SIZE: usize = 18;
pub const LINK_STATS_SIZE: usize = 5; // Only the first 4 fields.

pub const MAX_WAYPOINTS: usize = 30;
pub const WAYPOINT_SIZE: usize = F32_BYTES * 3 + WAYPOINT_MAX_NAME_LEN + 1;
pub const WAYPOINTS_SIZE: usize = MAX_WAYPOINTS * WAYPOINT_SIZE;
pub const SET_SERVO_POSIT_SIZE: usize = 1 + F32_BYTES;
pub const WAYPOINT_MAX_NAME_LEN: usize = 7;

// Packet sizes are payload size + 2. Additional data are message type, and CRC.
pub const PARAMS_PACKET_SIZE: usize = PARAMS_SIZE + 2;
pub const CONTROLS_PACKET_SIZE: usize = CONTROLS_SIZE + 2;
pub const LINK_STATS_PACKET_SIZE: usize = LINK_STATS_SIZE + 2;
pub const WAYPOINTS_PACKET_SIZE: usize = WAYPOINTS_SIZE + 2;

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
    LinkStats = 6,
    ReqLinkStats = 7,
    ArmMotors = 8,
    DisarmMotors = 9,
    StartMotor = 10,
    StopMotor = 11,
    ReqWaypoints = 12,
    Updatewaypoints = 13,
    Waypoints = 14,
    SetServoPosit = 15,
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
            Self::LinkStats => LINK_STATS_SIZE,
            Self::ReqLinkStats => 0,
            Self::ArmMotors => 0,
            Self::DisarmMotors => 0,
            Self::StartMotor => 1,
            Self::StopMotor => 1,
            Self::ReqWaypoints => 0,
            Self::Updatewaypoints => 10, // todo?
            Self::Waypoints => WAYPOINTS_SIZE,
            Self::SetServoPosit => SET_SERVO_POSIT_SIZE,
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
#[repr(u8)]
pub enum Rotor {
    R1 = 0,
    R2 = 1,
    R3 = 2,
    R4 = 3,
}

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum RotorPosition {
    FrontLeft = 0,
    FrontRight = 1,
    AftLeft = 2,
    AftRight = 3,
}

#[derive(Clone, Copy, Debug)]
pub enum ServoWing {
    S1,
    S2,
}

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum ServoWingPosition {
    Left = 0,
    Right = 1,
}

pub const fn crc_init(poly: u8) -> [u8; 256] {
    let mut lut = [0; 256];

    let mut i = 0;
    while i < 256 {
        // Can't use for loops in const fns
        let mut crc = i as u8;

        let mut j = 0;
        while j < 8 {
            crc = (crc << 1) ^ (if (crc & 0x80) > 0 { poly } else { 0 });
            j += 1;
        }
        lut[i] = crc;

        i += 1;
    }

    lut
}

/// CRC8 using a specific poly, includes all bytes from type (buffer[2]) to end of payload.
/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
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

#[derive(Clone, Copy, PartialEq)]
pub enum AircraftType {
    Quadcopter,
    FlyingWing,
}

#[derive(Clone, Default, Serialize)]
/// https://www.expresslrs.org/2.0/faq/#how-many-channels-does-elrs-support
pub struct LinkStats {
    /// Timestamp these stats were recorded. (TBD format; processed locally; not part of packet from tx).
    pub timestamp: u32,
    /// Uplink - received signal strength antenna 1 (RSSI). RSSI dBm as reported by the RX. Values
    /// vary depending on mode, antenna quality, output power and distance. Ranges from -128 to 0.
    pub uplink_rssi_1: u8,
    /// Uplink - received signal strength antenna 2 (RSSI).  	Second antenna RSSI, used in diversity mode
    /// (Same range as rssi_1)
    pub uplink_rssi_2: u8,
    /// Uplink - link quality (valid packets). The number of successful packets out of the last
    /// 100 from TX → RX
    pub uplink_link_quality: u8,
    /// Uplink - signal-to-noise ratio. SNR reported by the RX. Value varies mostly by radio chip
    /// and gets lower with distance (once the agc hits its limit)
    pub uplink_snr: i8,
    /// Active antenna for diversity RX (0 - 1)
    pub active_antenna: u8,
    pub rf_mode: u8,
    /// Uplink - transmitting power. (mW?) 50mW reported as 0, as CRSF/OpenTX do not have this option
    pub uplink_tx_power: u8,
    /// Downlink - received signal strength (RSSI). RSSI dBm of telemetry packets received by TX.
    pub downlink_rssi: u8,
    /// Downlink - link quality (valid packets). An LQ indicator of telemetry packets received RX → TX
    /// (0 - 100)
    pub downlink_link_quality: u8,
    /// Downlink - signal-to-noise ratio. 	SNR reported by the TX for telemetry packets
    pub downlink_snr: i8,
}

#[derive(Default, Clone, Serialize)]
pub struct Location {
    // Note: unlike Location in the main program, we ommit location type, and use String for name.
    pub name: String,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
