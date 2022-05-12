#![feature(proc_macro_hygiene, decl_macro)]
#![allow(non_snake_case)]

#[macro_use]
extern crate rocket;

use rocket::config::{Config, Environment, LoggingLevel};

use serde::Serialize;
use serde_json;

use rocket_contrib::serve::StaticFiles;

use std::{convert::TryInto, io, time::{Duration, Instant}};

use chrono;

use local_ipaddress;
use serialport::{self, SerialPortType};

mod from_firmware;

use from_firmware::*;

pub static mut PARAMS: Option<Params> = None;
pub static mut CONTROLS: Option<ChannelData> = None;
pub static mut LAST_PARAMS_UPDATE: Option<Instant> = None;
pub static mut LAST_CONTROLS_UPDATE: Option<Instant> = None;

const BAUD: u32 = 9_600;


// Code in this section is a reverse of buffer <--> struct conversion in `usb_cfg`.

impl From<[u8; PARAMS_SIZE]> for Params {
    /// 19 f32s x 4 = 76. In the order we have defined in the struct.
    fn from(p: [u8; PARAMS_SIZE]) -> Self {
        Params {
            s_x: bytes_to_float(&p[0..4]),
            s_y: bytes_to_float(&p[0..4]),
            s_z_msl: bytes_to_float(&p[0..4]),
            s_z_agl: bytes_to_float(&p[0..4]),
        
            s_pitch: bytes_to_float(&p[0..4]),
            s_roll: bytes_to_float(&p[0..4]),
            s_yaw: bytes_to_float(&p[0..4]),

            v_x: bytes_to_float(&p[0..4]),
            v_y: bytes_to_float(&p[0..4]),
            v_z: bytes_to_float(&p[0..4]),
        
            v_pitch: bytes_to_float(&p[0..4]),
            v_roll: bytes_to_float(&p[0..4]),
            v_yaw: bytes_to_float(&p[0..4]),
        
            a_x: bytes_to_float(&p[0..4]),
            a_y: bytes_to_float(&p[0..4]),
            a_z: bytes_to_float(&p[0..4]),
        
            a_pitch: bytes_to_float(&p[0..4]),
            a_roll: bytes_to_float(&p[0..4]),
            a_yaw: bytes_to_float(&p[0..4]),
        }

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
    ser: Box<dyn serialport::SerialPort>,
}

impl Fc {
    pub fn new() -> Result<Self, io::Error> {
        // if let Ok(ports) = serialport::available_ports() {
            // for port in &ports {
                // if let SerialPortType::UsbPort(info) = &port.port_type {
                //     if let Some(sn) = &info.serial_number {
                //         if sn == "an" {
                            let mut port = serialport::new("an", BAUD)
                                .open()
                                .expect("Failed to open serial port");


                            return Ok(Self {
                                ser: port,
                            });
                        // }
                    // }
                // }
            // }
        // }
        // Err(io::Error::new(
        //     io::ErrorKind::Other,
        //     "Can't get readings from the FC",
        // ))
    }

    pub fn read_all(&mut self) -> Result<Params, io::Error> {
        let crc = 0; // todo!
        let xmit_buf_params = &[MsgType::ReqParams as u8, crc]; // todo: Don't hard code it like this?
        let xmit_buf_controls = &[MsgType::ReqControls as u8, crc]; // todo: Don't hard code it like this?

        self.ser.write(xmit_buf_params)?;

        let mut rx_buf = [0; PARAMS_SIZE + 2];
        self.ser.read(&mut rx_buf)?;

        // todo: Msg type and CRC check.
        let mut payload = [0; PARAMS_SIZE];
        for i in 0..PARAMS_SIZE {
            payload[i] = rx_buf[i + 1];
        }

        Ok(payload.into())
    }

    /// Close the serial port
    pub fn close(&mut self) {}
}

/// Get readings over JSON, which we've cached.
#[get("/data")]
fn view_readings() -> String {
    let last_update = unsafe { LAST_PARAMS_UPDATE.as_ref().unwrap() };

    // Only update the readings from the WM if we're past the last updated thresh.
    if (Instant::now() - *last_update) > Duration::new(0, REFRESH_INTERVAL * 1_000_000) {
        if let Err(_) = get_data() {
            // todo: Is this normal? Seems harmless, but I'd like to
            // todo get to the bottom of it.
            // println!("Problem getting readings; sending old.")
        }

        unsafe { LAST_PARAMS_UPDATE = Some(Instant::now()) };
    }

    let readings = unsafe { &PARAMS.as_ref().unwrap() };
    return serde_json::to_string(readings).unwrap_or("Problem taking readings".into());
    // return serde_json::to_string(readings).unwrap_or("Problem taking readings".into());
}

/// Request readings from the FC over USB/serial. Cache them as a
/// global variable. Requesting the readings directly from the frontend could result in
/// conflicts, where multiple frontends are requesting readings from the WM directly
/// in too short an interval.
fn get_data() -> Result<(), io::Error> {
    let fc_ = Fc::new();

    if let Ok(mut fc) = fc_ {
        let params = fc.read_all().unwrap_or_default();
        fc.close();
        unsafe { PARAMS = Some(params) };
        Ok(())
    } else {
        Err(io::Error::new(
            io::ErrorKind::Other,
            "Can't find the flight controller.",
        ))
    }
}

fn main() {
    unsafe { PARAMS = Some(Default::default()) };
    unsafe { CONTROLS = Some(Default::default()) };
    unsafe { LAST_PARAMS_UPDATE = Some(Instant::now()) };
    unsafe { LAST_CONTROLS_UPDATE = Some(Instant::now()) };

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
        .mount("/api", routes![view_readings])
        .launch();
}
