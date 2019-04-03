

pub struct Control {
    port: serial::SystemPort,
    id: Option<usize>,
    a: i32,
    b: i32
}

impl Control {
    pub fn open(path: &str) -> Result<Control, serial::Error> {
        let mut port = serial::open(path)?;
        port.reconfigure(&|settings| {
            settings.set_baud_rate(serial::Baud9600)?;
            settings.set_char_size(serial::Bits8);
            settings.set_parity(serial::ParityNone);
            settings.set_stop_bits(serial::Stop1);
            settings.set_flow_control(serial::FlowNone);
            Ok(())
        })?;
        Ok(Control {
            port: port,
            id: None,
            a: 0,
            b: 0
        })
    }
    pub fn request_id(&mut self) {
        writeln!(self.port, "i");
    }
    pub fn slew(&mut self, time: f32, a: i32, b: i32) {
        let delta = 0.1;
        let mut remaining = time;
        let (ia, ib) = (self.a as f32, self.b as f32);
        while remaining > delta {
            let p = remaining / time;
            self.set((p*ia + (1.0-p)*(a as f32)) as i32, (p*ib + (1.0-p)*(b as f32)) as i32);
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
            let vals: Vec<&str> = st.split_whitespace().collect();
            if vals.len() > 0 && vals[0] == "id" {
                self.id = Some(vals[1].parse().unwrap());
            }
        }
    }
}
