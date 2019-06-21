use nalgebra::*;
use motor::Manager;

use std::thread;
use std::time::{Duration, Instant};

use std::sync::mpsc;

use std::f32::consts;

use crate::*;

pub enum ControllerCommand {
    Motion(Vector4<f32>),
    Rest
}

pub fn create_control_endpoint<M:Manager + Send + 'static>(mut mod_a: ModControl<M>, mut mod_b: ModControl<M>, config: ModuleConfig, half_life: f32, timeout: Duration) -> mpsc::Sender<ControllerCommand> {
    let (send, recv) = mpsc::channel::<ControllerCommand>();
    thread::spawn(move || {
        let mut control: Vector4<f32> = Vector4::zeros();
        let mut state: Vector4<f32> = [50.0, 50.0, 0.0, 0.0].into();
        let mut current_time = Instant::now();
        let mut last_set = current_time;
        let mut resting = true;

        'main_loop: loop {
            let tdiff = {
                let new_current = Instant::now();
                let diff = new_current.duration_since(current_time);
                current_time = new_current;
                diff
            };
            thread::sleep(Duration::from_millis(100));
            loop {
                match recv.try_recv() {
                    Ok(ControllerCommand::Motion(vec)) => {
                        resting = false;
                        control = vec;
                        last_set = current_time;
                    }
                    Ok(ControllerCommand::Rest) => {
                        resting = true;
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        break 'main_loop;
                    }
                    Err(_) => break,
                }
            }
            let fdiff = tdiff.as_secs() as f32 + (tdiff.subsec_millis() as f32) / 1000.0;
            if last_set.elapsed() > timeout {
                control *= ((-1.0) * fdiff / half_life).exp2();
            }
            state += control * fdiff;
            let cyl = Cyl {
                r: state[0],
                z: state[1],
                theta: consts::PI*state[2]/180.0,
            };
            let set_time = Instant::now();
            if resting {
                let empty_state = ModuleState::new();
                mod_a.set(empty_state.clone());
                mod_b.set(empty_state);
            } else {
                let states = sym_2mod_a(cyl, &config, 0.0);
                if let Some((s1, s2)) = states {
                    mod_a.set(s1);
                    mod_b.set(s2);
                }
            }
            let x = set_time.elapsed();
            //println!("{}  {}", x.as_secs(), x.subsec_millis());
        }
    });
    send
}


