//! This module contains types etc that are copy+pasted from the firmware.

pub static mut CRC_LUT: [u8; 256] = [0; 256];
pub const CRC_POLY: u8 = 0xab;

// pub const PARAMS_SIZE: usize = 76; // + message type, payload len, and crc.
pub const ATTITUDE_SIZE: usize = 16; // + message type, payload len, and crc.
pub const CONTROLS_SIZE: usize = 18; // + message type, payload len, and crc.

// pub const MAX_PAYLOAD_SIZE: usize = PARAMS_SIZE; // For Params.
pub const MAX_PAYLOAD_SIZE: usize = CONTROLS_SIZE; // For Params.
pub const MAX_PACKET_SIZE: usize = MAX_PAYLOAD_SIZE + 2; // + message type, payload len, and crc.

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

pub struct Packet {
    message_type: MsgType,
    payload: [u8; MAX_PAYLOAD_SIZE], // todo?
    crc: u8,
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
//
// impl Add<Self> for Quaternion {
//     type Output = Self;
//
//     fn add(self, rhs: Self) -> Self::Output {
//         Self {
//             w: self.w + rhs.w,
//             x: self.x + rhs.x,
//             y: self.y + rhs.y,
//             z: self.z + rhs.z,
//         }
//     }
// }
//
// impl Mul<Self> for Quaternion {
//     type Output = Self;
//
//     fn mul(self, rhs: Self) -> Self::Output {
//         Self {
//             w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
//             x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
//             y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
//             z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
//         }
//     }
// }
//
// impl Mul<Vec3> for Quaternion {
//     type Output = Self;
//
//     /// Returns the multiplication of a Quaternion with a vector.  This is a
//     /// normal Quaternion multiplication where the vector is treated a
//     /// Quaternion with a W element value of zero.  The Quaternion is post-
//     /// multiplied by the vector.
//     fn mul(self, rhs: Vec3) -> Self::Output {
//         Self {
//             w: -self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
//             x: self.w * rhs.x + self.y * rhs.z - self.z * rhs.y,
//             y: self.w * rhs.y - self.x * rhs.z + self.z * rhs.x,
//             z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x,
//         }
//     }
// }
//
// impl Quaternion {
//     pub fn new_identity() -> Self {
//         Self {
//             w: 1.,
//             x: 0.,
//             y: 0.,
//             z: 0.,
//         }
//     }
//
//     /// Converts a Quaternion to Euler angles, in radians.
//     pub fn to_euler(self) -> EulerAngle {
//         let half_minus_qy_squared = 0.5 - self.y * self.y; // calculate common terms to avoid repeated operations
//
//         EulerAngle {
//             roll: (self.w * self.x + self.y * self.z)
//                 .atan2(half_minus_qy_squared - self.x * self.x),
//             pitch: (2.0 * (self.w * self.y - self.z * self.x)).asin(),
//             yaw: (self.w * self.z - self.x * self.y).atan2(half_minus_qy_squared - self.z * self.z),
//         }
//     }
//
//     /// Converts a Quaternion to a rotation matrix
//     #[rustfmt::skip]
//     pub fn to_matrix(self) -> Mat3 {
//         let qwqw = self.w * self.w; // calculate common terms to avoid repeated operations
//         let qwqx = self.w * self.x;
//         let qwqy = self.w * self.y;
//         let qwqz = self.w * self.z;
//         let qxqy = self.x * self.y;
//         let qxqz = self.x * self.z;
//         let qyqz = self.y * self.z;
//
//         Mat3 {
//             data: [
//                 2.0 * (qwqw - 0.5 + self.x * self.x),
//                 2.0 * (qxqy + qwqz),
//                 2.0 * (qxqz - qwqy),
//                 2.0 * (qxqy - qwqz),
//                 2.0 * (qwqw - 0.5 + self.y * self.y),
//                 2.0 * (qyqz + qwqx),
//                 2.0 * (qxqz + qwqy),
//                 2.0 * (qyqz - qwqx),
//                 2.0 * (qwqw - 0.5 + self.z * self.z),
//             ]
//         }
//     }
//
//     pub fn magnitude(&self) -> f32 {
//         (self.w.powi(2) + self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
//     }

    // /// Returns the normalised quaternion
    // pub fn to_normalized(self) -> Self {
    //     // println!("Q w in mag fn: {}", self.w);
    //     // println!("Q mag: {}", self.magnitude());
    //     let mag_recip = 1. / self.magnitude();
    //
    //     Self {
    //         w: self.w * mag_recip,
    //         x: self.x * mag_recip,
    //         y: self.y * mag_recip,
    //         z: self.z * mag_recip,
    //     }
    // }
// }
