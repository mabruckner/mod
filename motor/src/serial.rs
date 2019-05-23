use std::io;
use std::io::{BufRead, BufReader, Write};
use std::thread;
use std::time::Duration;

use serial::SerialPort;

pub struct Control {
    port: serial::SystemPort,
    pub id: Option<usize>,
    a: i32,
    b: i32,
    a_pos: i32,
    b_pos: i32,
    a_err: i32,
    b_err: i32,
}

impl Control {
    pub fn open(path: &str) -> Result<Control, serial::Error> {
        let mut port = serial::open(path)?;
        port.reconfigure(&|settings| {
            settings.set_baud_rate(serial::Baud9600)?;
            settings.set_char_size(serial::Bits8);
            settings.set_parity(serial::ParityEven);
            settings.set_stop_bits(serial::Stop2);
            settings.set_flow_control(serial::FlowNone);
            Ok(())
        })?;
        Ok(Control {
            port: port,
            id: None,
            a: 0,
            b: 0,
            a_pos: 0,
            b_pos: 0,
            a_err: 0,
            b_err: 0,
        })
    }
    pub fn request_id(&mut self) {
        writeln!(self.port, "i");
    }
    pub fn request_status(&mut self) {
        writeln!(self.port, "s");
    }
    pub fn slew(&mut self, time: f32, a: i32, b: i32) {
        let delta = 0.1;
        let mut remaining = time;
        let (ia, ib) = (self.a as f32, self.b as f32);
        while remaining > delta {
            let p = remaining / time;
            self.set(
                (p * ia + (1.0 - p) * (a as f32)) as i32,
                (p * ib + (1.0 - p) * (b as f32)) as i32,
            );
            thread::sleep(Duration::from_millis(100));
            remaining -= delta;
        }
        self.set(a, b);
    }
    pub fn set(&mut self, a: i32, b: i32) {
        self.a = a;
        self.b = b;
        //let m1 = (20.0*24.0*(a - b)) as i16;
        //let m2 = (20.0*24.0*(a + b)) as i16;
        writeln!(self.port, "m{},{}", a, b);
        //self.port.write(&[b'm', (m1>>8) as u8, (m1&0xff) as u8, (m2>>8) as u8, (m2&0xff) as u8, b'\n']);
    }
    pub fn drain_serial(&mut self) -> () {
        let mut reader = BufReader::new(&mut self.port);
        let mut st = String::new();
        while reader.read_line(&mut st).is_ok() {
            let mut vals: Vec<&str> = st.split_whitespace().collect();
            let start = vals.remove(0);
            match (start, &vals[..]) {
                (Some("id"), [x]) => {
                    self.id = Some(x.parse().unwrap())
                },
                (Some("status"), vals) => {
                    let nums: Vec<i32> = (0..4).map(|x| x.parse().unwrap()).collect();
                    self.a_pos = nums[0];
                    self.a_err = nums[1];
                    self.b_pos = nums[2];
                    self.b_err = nums[3];
                }
            }
            if vals.len() > 0 && vals[0] == "id" {
                self.id = Some(vals[1].parse().unwrap());
                
            }
        }
    }
}
