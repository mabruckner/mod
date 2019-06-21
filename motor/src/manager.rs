#[cfg(feature="nonmanager")]
use crate::config::*;
#[cfg(feature="nonmanager")]
use crate::message::*;
#[cfg(feature="clientmanager")]
use hyper::rt::Future;
#[cfg(feature="clientmanager")]
use futures::stream::Stream;
#[cfg(feature="clientmanager")]
use futures::sink::Sink;
use std::thread;
use std::time;
use std::sync::mpsc;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::fs;

pub trait Manager {
    fn get_motors(&self) -> Vec<String>;
    fn set_motor(&mut self, name: &str, value: f32) -> Result<(), ()>;
    fn set_motor_rad(&mut self, name: &str, value: f32) -> Result<(), ()> {
        self.set_motor(name, value / (2.0 * std::f32::consts::PI))
    }
    fn get_servos(&self) -> Vec<String>;
    fn set_servo(&mut self, name: &str, value: f32) -> Result<(), ()>;
}

#[cfg(feature="nonmanager")]
struct DeviceLayer {
    config: DeviceConfig,
    boards: Vec<(PathBuf, Option<usize>, mpsc::Sender<BoardInput>, mpsc::Receiver<BoardOutput>)>,
}

#[cfg(feature="nonmanager")]
impl DeviceLayer {
    fn new(config: DeviceConfig) -> Self {
        DeviceLayer {
            config: config,
            boards: Vec::new()
        }
    }
    fn check_devices(&mut self) -> () {
        println!("{:?}", self.boards);
        let r = regex::Regex::new(r"^ttyA.*$").unwrap();
        let mut to_test = Vec::new();
        for entry in fs::read_dir("/dev").unwrap() {
            if let Ok(entry) = entry {
                if r.is_match(entry.file_name().to_str().unwrap()) {
                    to_test.push(entry.path());
                }
            }
        }
        for path in to_test {
            if self.boards.iter().filter(|x| x.0 == path).next().is_some() {
                // do nothing I guess
            } else {
                match bind_port_path(&path.to_str().unwrap()) {
                    Ok((r, s, _)) => {
                        s.send(BoardInput::GetID);
                        self.boards.push((path, None, s, r));
                    },
                    Err(e) => {
                        eprintln!("Unable to bind port: {:?}", e);
                    }
                }
            }
        }
    }
    fn get_motor(&self, name: &str) -> Option<(usize, mpsc::Sender<BoardInput>, MotorSelect)> {
        for board in &self.boards {
            if let Some(Some(conf)) = board.1.map(|id| self.config.boards.get(&id)) {
                if conf.a.name == name {
                    return Some((board.1.unwrap(), board.2.clone(), MotorSelect::A));
                }
                if conf.b.name == name {
                    return Some((board.1.unwrap(), board.2.clone(), MotorSelect::B));
                }
            }
        }
        None
    }
    fn get_servo(&self, name: &str) -> Option<(&ServoConfig, mpsc::Sender<BoardInput>)> {
        for board in &self.boards {
            if let Some(Some(conf)) = board.1.map(|id| self.config.boards.get(&id)) {
                if conf.servo.name == name {
                    return Some((&conf.servo, board.2.clone()));
                }
            }
        }
        None
    }
    fn set_motor(&self, name: &str, val: f32) -> Result<(), ()> {
        let (id, send, select) = self.get_motor(name).ok_or(())?;
        let conf = match select {
            MotorSelect::A => &self.config.boards.get(&id).unwrap().a,
            MotorSelect::B => &self.config.boards.get(&id).unwrap().b,
        };
        let counts = (val * conf.counts_per_rev).round() as i16;
        send.send(BoardInput::SetMotor(select, counts));
        Ok(())
    }
    fn set_servo(&self, name: &str, val: f32) -> Result<(), ()> {
        let (conf, send) = self.get_servo(name).ok_or(())?;
        let a = conf.extents.0 as f32;
        let b = conf.extents.1 as f32;
        let mapped = a * (1.0 - val) + b * val;
        send.send(BoardInput::SetServo(mapped.round() as u8));
        Ok(())
    }
    fn consume_output(&mut self) -> () {
        for i in (0..self.boards.len()).rev() {
            'boardloop: loop {
                match self.boards[i].3.try_recv() {
                    Ok(BoardOutput::ID(id)) => {
                        self.boards[i].1 = Some(id as usize)
                    },
                    Ok(_) => (),
                    Err(mpsc::TryRecvError::Empty) => break 'boardloop,
                    Err(mpsc::TryRecvError::Disconnected) => {
                        self.boards.remove(i);
                        break 'boardloop;
                    }
                }
            }
        }
    }
    fn poll_id(&mut self) -> () {
        for board in &mut self.boards {
            if board.1.is_none() {
                board.2.send(BoardInput::GetID);
            }
        }
    }
}

#[cfg(feature="nonmanager")]
pub struct DirectManager {
    layer: Arc<Mutex<DeviceLayer>>
}

#[cfg(feature="nonmanager")]
impl DirectManager {
    pub fn new(config: DeviceConfig) -> Self {
        let mut layer = DeviceLayer::new(config);
        let layer = Arc::new(Mutex::new(layer));
        let weak = Arc::downgrade(&layer);
        thread::spawn(move || {
            while let Some(layer) = weak.upgrade() {
                {
                    let mut dev = layer.lock().unwrap();
                    dev.check_devices();
                    dev.consume_output();
                }
                thread::sleep(time::Duration::from_millis(1000));
            }
        });
        DirectManager {
            layer: layer
        }
    }
}

#[cfg(feature="nonmanager")]
impl Manager for DirectManager {
    fn get_motors(&self) -> Vec<String> {
        let dev = self.layer.lock().unwrap();
        let mut output = Vec::new();
        for board in &dev.boards {
            let result = board.1.map(|id| dev.config.boards.get(&id));
            match result {
                Some(Some(conf)) => { // conf is of type &BoardConfig
                    output.push(conf.a.name.clone());
                    output.push(conf.b.name.clone());
                },
                _ => ()
            }
        }
        output
    }
    fn get_servos(&self) -> Vec<String> {
        let dev = self.layer.lock().unwrap();
        let mut output = Vec::new();
        for board in &dev.boards {
            let result = board.1.map(|id| dev.config.boards.get(&id));
            match result {
                Some(Some(conf)) => { // conf is of type &BoardConfig
                    output.push(conf.servo.name.clone());
                },
                _ => ()
            }
        }
        output
    }
    fn set_motor(&mut self, name: &str, val: f32) -> Result<(), ()> {
        let dev = self.layer.lock().unwrap();
        dev.set_motor(name, val)
    }
    fn set_servo(&mut self, name: &str, val: f32) -> Result<(), ()> {
        let dev = self.layer.lock().unwrap();
        dev.set_servo(name, val)
    }
}

impl Manager for () {
    fn get_motors(&self) -> Vec<String> {
        Vec::new()
    }
    fn get_servos(&self) -> Vec<String> {
        Vec::new()
    }
    fn set_motor(&mut self, name: &str, val: f32) -> Result<(), ()> {
        Err(())
    }
    fn set_servo(&mut self, name: &str, val: f32) -> Result<(), ()> {
        Err(())
    }
}


#[cfg(feature="clientmanager")]
pub struct ClientManager {
    servers: Vec<hyper::Uri>,
    client: hyper::Client<hyper::client::HttpConnector, hyper::Body>,
    futures: Vec<hyper::client::ResponseFuture>,
    sender: tokio::sync::mpsc::Sender<(String, f32)>
}

#[cfg(feature="clientmanager")]
impl ClientManager {
    pub fn new(servers: Vec<String>) -> Self {
        let servers: Vec<hyper::Uri> = servers.iter().map(|x| x.parse::<hyper::Uri>().unwrap()).collect();
        let man_servers = servers.clone();
        let (send, recv) = tokio::sync::mpsc::channel(8);
        thread::spawn(move || {
            hyper::rt::run(hyper::rt::lazy(move || {
                let client = hyper::Client::new();
                hyper::rt::spawn(recv.for_each(move |(name, value)| {
                    for server in &servers {
                        let target: hyper::Uri = format!("{}motor?name={}&value={}", server, name, value).parse().unwrap();
                        let start = time::Instant::now();
                        hyper::rt::spawn(client.get(target).map(move |x| {
                            println!("{:?}", start.elapsed());
                            ()
                        }).map_err(|x| {
                            //println!("{:?}", x);
                            ()
                        }));
                    }
                    Ok(())
                }).map_err(|_| ()));
                Ok(())
            }))
        });
        ClientManager {
            servers: man_servers,
            sender: send,
            client: hyper::Client::new(),
            futures: Vec::new()
        }
    }
    fn drain_futures(&mut self) -> () {
        for i in (0..self.futures.len()).rev() {
            let val = self.futures[i].poll();
            match val {
                Ok(futures::Async::NotReady) => {
                self.futures.swap_remove(i);
                },
                _ => ()
            }
        }
    }
}

#[cfg(feature="clientmanager")]
impl Manager for ClientManager {
    fn get_motors(&self) -> Vec<String> {
        let mut out = Vec::new();
        for server in &self.servers {
            if let Ok(mut data) = self.client.get(format!("{}/motors", server).parse().unwrap()).wait() {
                /*if let Ok(lst) = data.json::<Vec<String>>() {
                    out.extend(lst.into_iter());
                }*/
                unimplemented!();
            }
        }
        out
    }
    fn get_servos(&self) -> Vec<String> {
        let mut out = Vec::new();
        for server in &self.servers {
            if let Ok(mut data) = reqwest::get(&format!("{}/motors", server)) {
                if let Ok(lst) = data.json::<Vec<String>>() {
                    out.extend(lst.into_iter());
                }
            }
        }
        out
    }
    fn set_motor(&mut self, name: &str, val: f32) -> Result<(), ()> {
        self.sender.try_send((name.into(), val));
        Ok(())
    }
    fn set_servo(&mut self, name: &str, val: f32) -> Result<(), ()> {
        for server in &self.servers {
            reqwest::get(&format!("{}/servo?name={}&value={}", server, name, val));
        }
        Ok(())
    }
}
