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
pub static mut ATTITUDE: Quaternion = Quaternion { w: 0., x: 0., y: 0., z: 0. };
pub static mut CONTROLS: Option<ChannelData> = None;
pub static mut LAST_PARAMS_UPDATE: Option<Instant> = None;
pub static mut LAST_CONTROLS_UPDATE: Option<Instant> = None;

const FC_SERIAL_NUMBER: &'static str = "AN";

const BAUD: u32 = 9_600;

/// Convert radians to degrees
fn to_degrees(v: f32) -> f32 {
    v * 360. / TAU
}

#[derive(Serialize)]
struct ReadData {
    // params: Params,
    attitude: Quaternion,
    controls: ChannelData,
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

impl From<[u8; ATTITUDE_SIZE]> for Quaternion {
    /// 4 f32s = 16. In the order we have defined in the struct.
    fn from(p: [u8; ATTITUDE_SIZE]) -> Self {
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
/// Copy+pasted from `water_monitor::util`
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
                            // println!("Port: {:?}", port_info);
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

    // pub fn read_all(&mut self) -> Result<(Params, ChannelData), io::Error> {
    pub fn read_all(&mut self) -> Result<(Quaternion, ChannelData), io::Error> {
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

        let xmit_buf_params = &[MsgType::ReqParams as u8, crc_tx_params];
        let xmit_buf_controls = &[MsgType::ReqControls as u8, crc_tx_controls];

        self.ser.write(xmit_buf_params)?;

        // let mut rx_buf = [0; PARAMS_SIZE + 2];
        let mut rx_buf = [0; ATTITUDE_SIZE + 2];
        self.ser.read(&mut rx_buf)?;

        // todo: Msg type and CRC check.
        // let mut payload_params = [0; PARAMS_SIZE];
        // for i in 0..PARAMS_SIZE {
        //     payload_params[i] = rx_buf[i + 1];
        // }
        //
        let mut payload_attitude = [0; ATTITUDE_SIZE];
        for i in 0..ATTITUDE_SIZE {
            payload_attitude[i] = rx_buf[i + 1];
        }


        self.ser.write(xmit_buf_controls)?;

        // let mut rx_buf = [0; CONTROLS_SIZE + 2]; // todo: Bogus leading 1?
        let mut rx_buf = [0; CONTROLS_SIZE + 3];
        self.ser.read(&mut rx_buf)?;

        // println!("RX buf: {:?}", rx_buf);

        let mut payload_controls = [0; CONTROLS_SIZE];
        for i in 0..CONTROLS_SIZE {
            // payload_controls[i] = rx_buf[i + 1];  // todo: Bogus leading 1?
            payload_controls[i] = rx_buf[i + 2];
        }

        // println!("Payload ctrls: {:?}", payload_controls);

        let payload_size = MsgType::ReqParams.payload_size();
        // let crc_rx_expected = calc_crc(
        //     unsafe { &CRC_LUT },
        //     &rx_buf[..payload_size + 1],
        //     payload_size as u8 + 1,
        // );

        // Ok((payload_params.into(), payload_controls.into()))
        Ok((payload_attitude.into(), payload_controls.into()))
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

    println!("Attitude; {}", attitude);

    let data = unsafe { ReadData {
        // params: PARAMS.clone().unwrap(),
        attitude: ATTITUDE,
        controls:CONTROLS.clone().unwrap(),
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
        // let (params, controls) = fc.read_all().unwrap_or_default();
        let (attitude, controls) = fc.read_all().unwrap_or_default();

        fc.close();

        // unsafe { PARAMS = Some(params) };
        unsafe { ATTITUDE = attitude };
        unsafe { CONTROLS = Some(controls) };
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
    unsafe { CONTROLS = Some(Default::default()) };
    unsafe { LAST_PARAMS_UPDATE = Some(Instant::now()) };
    unsafe { LAST_CONTROLS_UPDATE = Some(Instant::now()) };

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
