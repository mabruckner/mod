use serial;
use serial::SerialPort;
use std::sync::mpsc;
use std::time;
use std::thread;
use std::io;
use std::io::{BufRead, Write};
use std::path::Path;

pub fn bind_port_path(path: &str) -> Result<(mpsc::Receiver<BoardOutput>, mpsc::Sender<BoardInput>, thread::JoinHandle<()>), serial::Error> {
    let mut port = serial::open(path)?;
    bind_port(port)
}

pub fn bind_port(mut port: serial::SystemPort) -> Result<(mpsc::Receiver<BoardOutput>, mpsc::Sender<BoardInput>, thread::JoinHandle<()>), serial::Error> {
    port.reconfigure(&|settings| {
        settings.set_baud_rate(serial::Baud9600)?;
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityEven);
        settings.set_stop_bits(serial::Stop2);
        settings.set_flow_control(serial::FlowNone);
        Ok(())
    })?;
    let (s_in, r_in) = mpsc::channel::<BoardInput>();
    let (s_out, r_out) = mpsc::channel::<BoardOutput>();
    let mut reader = io::BufReader::new(port);

    let handle = thread::spawn(move || {
        'mainloop: loop {
            'readloop: loop {
                let mut buf = String::new();
                match reader.read_line(&mut buf) {
                    Ok(0) => break 'mainloop,
                    Ok(_) => {
                        let mut vals: Vec<&str> = buf.split_whitespace().collect();
                        println!("{:?}", vals);
                        let start = if vals.len() > 0 {
                            Some(vals.remove(0))
                        } else {
                            None
                        };
                        let message = match(start, &vals[..]) {
                            (Some("id"), [x]) => {
                                x.parse::<u8>().map(|id| BoardOutput::ID(id)).ok()
                            },
                            (Some("status"), vals) => {
                                let parsed: Result<Vec<i16>, _> = vals.into_iter().map(|val| val.parse::<i16>()).collect();
                                if let Ok(nums) = parsed {
                                    if nums.len() >= 10 {
                                        let mut switches = [false; 4];
                                        for i in 0..4 {
                                            switches[i] = nums[i+6] != 0;
                                        }
                                        Some(BoardOutput::Status {
                                            a: MotionStatus {
                                                position: nums[0],
                                                error: nums[1] as u16,
                                                inv_vel: nums[2],
                                            },
                                            b: MotionStatus {
                                                position: nums[3],
                                                error: nums[4] as u16,
                                                inv_vel: nums[5],
                                            },
                                            switches: switches
                                        })
                                    } else {
                                        None
                                    }
                                } else {
                                    None
                                }
                            },
                            (Some("ready"), _) => {
                                Some(BoardOutput::Ready)
                            },
                            _ => {
                                None
                            }
                        };
                        if let Some(msg) = message {
                            s_out.send(msg);
                        }
                    },
                    Err(e) => {
                        break 'readloop;
                    }
                }
            }
            let mut port = reader.get_mut();
            'writeloop: loop {
                match r_in.try_recv() {
                    Ok(msg) => {
                        match msg {
                            BoardInput::GetID => writeln!(port, "i"),
                            BoardInput::GetStatus => writeln!(port, "s"),
                            BoardInput::SetServo(x) => writeln!(port, "x {}", x),
                            BoardInput::SetMotors(a, b) => writeln!(port, "m {} {}", a, b),
                            BoardInput::SetID(x) => writeln!(port, "d {}", x)
                        };
                    },
                    Err(mpsc::TryRecvError::Empty) => {
                        break 'writeloop
                    },
                    Err(mpsc::TryRecvError::Disconnected) => {
                        break 'mainloop
                    }

                }
            }
            thread::sleep(time::Duration::from_millis(50));
        }
    });


    Ok((r_out, s_in, handle))
}

#[derive(Debug)]
pub enum BoardInput {
    GetID,
    GetStatus,
    SetServo(u8),
    SetMotors(i16, i16),
    SetID(u8)
}

#[derive(Debug)]
pub struct MotionStatus {
    inv_vel: i16,
    error: u16,
    position: i16,
}

#[derive(Debug)]
pub enum BoardOutput {
    Status {
        a: MotionStatus,
        b: MotionStatus,
        switches: [bool; 4]
    },
    ID(u8),
    Ready,
}
