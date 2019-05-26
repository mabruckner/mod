use std::env::args;
use std::sync::mpsc;
use std::thread;
use std::time;

use motor::*;

fn main() {
    let path = args().nth(1).unwrap();
    let (recv, send, _) = bind_port_path(&path).unwrap();
    'mainloop: loop {
        println!("HELLO");
        loop {
            match recv.try_recv() {
                Ok(val) => println!("{:?}", val),
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => break 'mainloop,
            }
        }
        thread::sleep(time::Duration::from_millis(500));
        send.send(BoardInput::GetStatus);
    }
    println!("disconnected");


}
