use std::env::args;
use std::sync::mpsc;
use std::thread;
use std::time;

use motor::*;

fn main() {
    let path = args().nth(1).unwrap();
    let (recv, send, _) = bind_port_path(&path).unwrap();
    let mut t: f32 = 0.0;
    println!("HELLO");
    'mainloop: loop {
        loop {
            match recv.try_recv() {
                Ok(val) => println!("{:?}", val),
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => break 'mainloop,
            }
        }
        thread::sleep(time::Duration::from_millis(50));
        t += 0.05;
        send.send(BoardInput::SetServo((((t*2.0).sin()+1.0)*20.0) as u8 + 10));
    }
    println!("disconnected");


}
