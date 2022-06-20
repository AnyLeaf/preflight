#![feature(proc_macro_hygiene, decl_macro)]
#![allow(non_snake_case)]

#[macro_use]
extern crate rocket;

use rocket::{
    Outcome::*, Request,
    data::{Outcome, Data, FromDataSimple},
    config::{Config, Environment, LoggingLevel}
};

use serde::Serialize;
use serde_json;

use rocket_contrib::serve::StaticFiles;

use std::{
    convert::TryInto,
    f32::consts::TAU,
    io,
    time::{Duration, Instant},
};
use std::io::Read;

use chrono;

use local_ipaddress;
use serialport::{self, SerialPortType};

mod from_firmware;

use from_firmware::*;

// pub static mut PARAMS: Option<Params> = None;
// todo: Don't make this static muts. Find some other way.
pub static mut ATTITUDE: Quaternion = Quaternion { w: 0., x: 0., y: 0., z: 0. };
pub static mut CONTROLS: Option<ChannelData> = None;
pub static mut LINK_STATS: Option<LinkStats> = None;
pub static mut ALTIMETER: f32 = 0.;
pub static mut BATT_V: f32 = 0.;
pub static mut CURRENT: f32 = 0.;

pub static mut LAST_PARAMS_UPDATE: Option<Instant> = None;
pub static mut LAST_CONTROLS_UPDATE: Option<Instant> = None;
pub static mut LAST_LINK_STATS_UPDATE: Option<Instant> = None;

pub static mut AIRCRAFT_TYPE: AircraftType = AircraftType::Quadcopter;

const FC_SERIAL_NUMBER: &'static str = "AN";

const BAUD: u32 = 9_600;

/// Convert radians to degrees
fn to_degrees(v: f32) -> f32 {
    v * 360. / TAU
}

#[derive(Serialize, Default)]
struct ReadData {
    attitude_quat: Quaternion,
    altimeter: f32,
    batt_v: f32,
    current: f32,
    controls: ChannelData,
    link_stats: LinkStats,
}

// Code in this section is a reverse of buffer <--> struct conversion in `usb_cfg`.

// impl From<[u8; PARAMS_SIZE]> for Params {
//     /// 19 f32s x 4 = 76. In the order we have defined in the struct.
//     fn from(p: [u8; PARAMS_SIZE]) -> Self {
//         Params {
//             s_x: bytes_to_float(&p[0..4]),
//             s_y: bytes_to_float(&p[4..8]),
//             s_z_msl: bytes_to_float(&p[8..12]),
//             s_z_agl: bytes_to_float(&p[12..16]),
//
//             s_pitch: bytes_to_float(&p[16..20]),
//             s_roll: bytes_to_float(&p[20..24]),
//             s_yaw: bytes_to_float(&p[24..28]),
//
//             quaternion: bytes_to_float(&p[24..28]),
//
//             v_x: bytes_to_float(&p[28..32]),
//             v_y: bytes_to_float(&p[32..36]),
//             v_z: bytes_to_float(&p[36..40]),
//
//             v_pitch: bytes_to_float(&p[40..44]),
//             v_roll: bytes_to_float(&p[44..48]),
//             v_yaw: bytes_to_float(&p[48..52]),
//
//             a_x: bytes_to_float(&p[52..56]),
//             a_y: bytes_to_float(&p[56..60]),
//             a_z: bytes_to_float(&p[60..64]),
//
//             a_pitch: bytes_to_float(&p[64..68]),
//             a_roll: bytes_to_float(&p[68..72]),
//             a_yaw: bytes_to_float(&p[72..76]),
//         }
//     }
// }

impl From<[u8; QUATERNION_SIZE]> for Quaternion {
    /// 4 f32s = 16. In the order we have defined in the struct.
    fn from(p: [u8; QUATERNION_SIZE]) -> Self {
        Quaternion {
            w: bytes_to_float(&p[0..4]),
            x: bytes_to_float(&p[4..8]),
            y: bytes_to_float(&p[8..12]),
            z: bytes_to_float(&p[12..16]),
        }
    }
}

impl From<[u8; CONTROLS_SIZE]> for ChannelData {
    /// 19 f32s x 4 = 76. In the order we have defined in the struct.
    fn from(p: [u8; CONTROLS_SIZE]) -> Self {
        ChannelData {
            pitch: bytes_to_float(&p[0..4]),
            roll: bytes_to_float(&p[4..8]),
            yaw: bytes_to_float(&p[8..12]),
            throttle: bytes_to_float(&p[12..16]),

            arm_status: p[16].try_into().unwrap(),
            input_mode: p[17].try_into().unwrap(),

        }
    }
}

impl From<[u8; LINK_STATS_SIZE]> for LinkStats {
    /// 4 f32s = 16. In the order we have defined in the struct.
    fn from(p: [u8; LINK_STATS_SIZE]) -> Self {
        LinkStats {
            //     uplink_rssi_1: bytes_to_float(&p[0..4]),
            //     uplink_rssi_2: bytes_to_float(&p[4..8]),
            //     uplink_link_quality: bytes_to_float(&p[8..12]),
            //     uplink_snr: bytes_to_float(&p[12..16]),

            uplink_rssi_1: p[0],
            uplink_rssi_2: p[0],
            uplink_link_quality: p[0],
            uplink_snr: p[0] as i8,
            ..Default::default() // other fields not used.
        }
    }
}

impl FromDataSimple for RotorPosition {
    type Error = String;

    fn from_data(req: &Request, data: Data) -> Outcome<Self, String> {
        // Ensure the content type is correct before opening the data.
        // if req.content_type() != Some(&person_ct) {
        //     return Outcome::Forward(data);
        // }

        let mut contents = String::new();
        data.open().read_to_string(&mut contents).unwrap();

        Success(match contents.as_ref() {
            "front-left" => Self::FrontLeft,
            "front-right" => Self::FrontRight,
            "aft-left" => Self::AftLeft,
            "aft-right" => Self::AftRight,
            _ => panic!("Invalid motor passed from the frontend."),
        })
    }
}

// End code reversed from `quadcopter`.

// todo: Baud cfg?

// pub enum SerialError {};

/// Convert bytes to a float
pub fn bytes_to_float(bytes: &[u8]) -> f32 {
    let bytes: [u8; 4] = bytes.try_into().unwrap();
    f32::from_bits(u32::from_be_bytes(bytes))
}

/// This mirrors that in the Python driver
struct Fc {
    pub ser: Box<dyn serialport::SerialPort>,
}

impl Fc {
    pub fn new() -> Result<Self, io::Error> {
        if let Ok(ports) = serialport::available_ports() {
            for port_info in &ports {
                if let SerialPortType::UsbPort(info) = &port_info.port_type {
                    if let Some(sn) = &info.serial_number {
                        if sn == FC_SERIAL_NUMBER {
                            let port = serialport::new(&port_info.port_name, BAUD)
                                .open()
                                .expect("Failed to open serial port");

                            return Ok(Self { ser: port });
                        }
                    }
                }
            }
        }

        Err(io::Error::new(
            io::ErrorKind::Other,
            "Unable to connect to the flight controller.",
        ))
    }

    /// Request several types of data from the flight controller over USB serial. Return a struct
    /// containing the data.
    pub fn read_all(&mut self) -> Result<ReadData, io::Error> {
        let mut result = ReadData::default();

        let crc_tx_params = calc_crc(
            unsafe { &CRC_LUT },
            &[MsgType::ReqParams as u8],
            MsgType::ReqParams.payload_size() as u8 + 1,
        );
        let crc_tx_controls = calc_crc(
            unsafe { &CRC_LUT },
            &[MsgType::ReqControls as u8],
            MsgType::ReqControls.payload_size() as u8 + 1,
        );
        let crc_tx_link_stats = calc_crc(
            unsafe { &CRC_LUT },
            &[MsgType::ReqLinkStats as u8],
            MsgType::ReqLinkStats.payload_size() as u8 + 1,
        );

        let xmit_buf_params = &[MsgType::ReqParams as u8, crc_tx_params];
        let xmit_buf_controls = &[MsgType::ReqControls as u8, crc_tx_controls];
        let xmit_buf_link_stats = &[MsgType::ReqLinkStats as u8, crc_tx_link_stats];

        self.ser.write(xmit_buf_params)?;

        let mut rx_buf = [0; PARAMS_SIZE + 2];
        self.ser.read(&mut rx_buf)?;

        let attitude_data: [u8; QUATERNION_SIZE] = rx_buf[1..QUATERNION_SIZE + 1].try_into().unwrap();
        result.attitude_quat = attitude_data.into();

        self.ser.write(xmit_buf_controls)?;

        // let mut rx_buf = [0; CONTROLS_SIZE + 2]; // todo: Bogus leading 1?
        let mut rx_buf = [0; CONTROLS_SIZE + 2];
        self.ser.read(&mut rx_buf)?;

        let controls_data: [u8; CONTROLS_SIZE] = rx_buf[1..CONTROLS_SIZE + 1].try_into().unwrap();
        result.controls = controls_data.into();

        // todo: DRY between these calls
        self.ser.write(xmit_buf_link_stats)?;

        let mut rx_buf = [0; LINK_STATS_SIZE + 2];
        self.ser.read(&mut rx_buf)?;

        let link_stats_data: [u8; LINK_STATS_SIZE] = rx_buf[1..LINK_STATS_SIZE + 1].try_into().unwrap();
        result.link_stats = link_stats_data.into();

        // println!("Payload ctrls: {:?}", payload_controls);

        let payload_size = MsgType::ReqParams.payload_size();
        // let crc_rx_expected = calc_crc(
        //     unsafe { &CRC_LUT },
        //     &rx_buf[..payload_size + 1],
        //     payload_size as u8 + 1,
        // );

        Ok(result)
    }

    pub fn send_arm_command(&mut self) -> Result<(), io::Error> {
        let msg_type = MsgType::ArmMotors;
        let crc = calc_crc(
            unsafe { &CRC_LUT },
            &[msg_type as u8],
            msg_type.payload_size() as u8 + 1,
        );
        let xmit_buf = &[msg_type as u8, crc];
        self.ser.write(xmit_buf)?;

        Ok(())
    }

    pub fn send_disarm_command(&mut self) -> Result<(), io::Error> {
        let msg_type = MsgType::DisarmMotors;
        let crc = calc_crc(
            unsafe { &CRC_LUT },
            &[msg_type as u8],
            msg_type.payload_size() as u8 + 1,
        );
        let xmit_buf = &[msg_type as u8, crc];
        self.ser.write(xmit_buf)?;

        Ok(())
    }

    /// Close the serial port
    pub fn close(&mut self) {}
}

/// Get readings over JSON upon request from the browser, which we've cached.
#[get("/data")]
fn send_data() -> String {
    let last_update = unsafe { LAST_PARAMS_UPDATE.as_ref().unwrap() };

    // Only update the readings from the FC if we're past the last updated thresh.
    if (Instant::now() - *last_update) > Duration::new(0, REFRESH_INTERVAL * 1_000_000) {
        if let Err(_) = get_data() {
            // todo: Is this normal? Seems harmless, but I'd like to
            // todo get to the bottom of it.
            // println!("Problem getting readings; sending old.")
        }

        unsafe { LAST_PARAMS_UPDATE = Some(Instant::now()) };
    }

    // let params = unsafe { &PARAMS.as_ref().unwrap() };
    // let controls = unsafe { &CONTROLS.as_ref().unwrap() };

    // unsafe {
    //     println!("Attitude: {} {}", ATTITUDE.w, ATTITUDE.x);
    // }

    // todo: Better way than these globals?
    let data = unsafe { ReadData {
        // params: PARAMS.clone().unwrap(),
        attitude_quat: ATTITUDE,
        altimeter: ALTIMETER,
        batt_v: BATT_V,
        current: CURRENT,
        controls: CONTROLS.clone().unwrap(),
        link_stats: LINK_STATS.clone().unwrap(),
    } };

    return serde_json::to_string(&data).unwrap_or("Problem serializing data".into());
}

/// Arm all motors, for testing.
#[post("/arm_motors")]
fn arm_motors() -> Result<(), io::Error> {
    println!("Arming motors...");

    // todo: fc_ should probably be a global of some sort.
    // todo: DRY!
    let fc_ = Fc::new();

    if let Ok(mut fc) = fc_ {
        fc.send_arm_command();

        fc.close();

        Ok(())
    } else {
        Err(io::Error::new(
            io::ErrorKind::Other,
            "Can't find the flight controller.",
        ))
    }
}

/// Start a motor.
#[post("/start_motor", data = "<data>")]
fn start_motor(data: RotorPosition) -> Result<(), io::Error> {
    println!("Starting motor {:?}", data);

    // todo: fc_ should probably be a global of some sort.
    // todo: DRY!
    let fc_ = Fc::new();

    if let Ok(mut fc) = fc_ {
        fc.send_disarm_command();

        fc.close();

        Ok(())
    } else {
        Err(io::Error::new(
            io::ErrorKind::Other,
            "Can't find the flight controller.",
        ))
    }
}




/// Request readings from the FC over USB/serial. Cache them as a
/// global variable. Requesting the readings directly from the frontend could result in
/// conflicts, where multiple frontends are requesting readings from the WM directly
/// in too short an interval.
fn get_data() -> Result<(), io::Error> {

    // todo: fc_ should probably be a global of some sort.
    let fc_ = Fc::new();

    if let Ok(mut fc) = fc_ {
        let data = fc.read_all().unwrap_or_default();

        fc.close();

        unsafe {
            ATTITUDE = data.attitude_quat;
            ALTIMETER = data.altimeter;
            BATT_V = data.batt_v;
            CURRENT = data.current;
            CONTROLS = Some(data.controls);
            LINK_STATS = Some(data.link_stats);
        };

        Ok(())
    } else {
        Err(io::Error::new(
            io::ErrorKind::Other,
            "Can't find the flight controller.",
        ))
    }
}

fn main() {
    // unsafe { PARAMS = Some(Default::default()) };
    unsafe {
        CONTROLS = Some(Default::default());
        LAST_PARAMS_UPDATE = Some(Instant::now());
        LAST_CONTROLS_UPDATE = Some(Instant::now());
        CONTROLS = Some(Default::default());
        LINK_STATS = Some(Default::default());
    }

    crc_init(unsafe { &mut CRC_LUT }, CRC_POLY);

    println!(
        "AnyLeaf Preflight has launched. You can connect by opening `localhost` in a \
    web browser on this computer, or by navigating to `{}` on another device on this network, \
    like your phone.\n",
        local_ipaddress::get().unwrap_or("(Problem finding IP address)".into())
    );

    let config = Config::build(Environment::Staging)
        // .address("1.2.3.4")
        .port(80) // 80 means default, ie users can just go to localhost
        .log_level(LoggingLevel::Critical) // Don't show the user the connections.
        .finalize()
        .expect("Problem setting up our custom config");

    rocket::custom(config)
        .mount("/", StaticFiles::from("static"))
        .mount("/api", routes![send_data, arm_motors, start_motor])
        .launch();
}
