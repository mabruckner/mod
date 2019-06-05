extern crate kiss3d;
extern crate nalgebra;
extern crate serial;

use motor::Manager;
//extern crate actix_web;

use kiss3d::light::Light;
use kiss3d::window::Window;
use nalgebra::*;

//use actix_web::{web, App, HttpServer, Responder};

use std::f32::consts;

use std::thread;
use std::time::Duration;

use std::io;
use std::io::{BufRead, BufReader, Write};

use std::sync::{mpsc, Arc, Mutex};

mod control;
use control::*;

mod module;
use module::*;

mod arm;
use arm::*;

mod controller;
use controller::*;

fn command<M:Manager>(mod_a: &mut ModControl<M>, mod_b: &mut ModControl<M>) {
    let config = ModuleConfig {
        axial_cprad: 100.0,
        hinge_cprad: 100.0,
        body_len: 35.0,
        hinge_offset: 7.5,
        diff_offset: 10.0,
    };
    let stdin = io::stdin();
    let handle = stdin.lock();
    for line in handle.lines() {
        if let Ok(line) = line {
            let mut split = line.split_whitespace();
            if let Some(command) = split.next() {
                let args: Result<Vec<f32>, _> = split.map(|x| x.parse::<f32>()).collect();
                if let Ok(args) = args {
                    match (command, args.as_slice()) {
                        ("cyl", &[r, z, theta]) => {
                            let states = sym_2mod_a(
                                Cyl {
                                    z: z,
                                    r: r,
                                    theta: consts::PI * theta / 180.0,
                                },
                                &config,
                                0.0,
                            );
                            if let Some((s1, s2)) = states {
                                mod_a.set(s1);
                                mod_b.set(s2);
                            } else {
                                println!("Out of bounds");
                            }
                        }
                        ("set_a", &[f, n, a, h]) => {
                            mod_a.set(ModuleState {
                                far_diff: f,
                                near_diff: n,
                                axial: a,
                                hinge: h,
                            });
                        }
                        ("set_b", &[f, n, a, h]) => {
                            mod_b.set(ModuleState {
                                far_diff: f,
                                near_diff: n,
                                axial: a,
                                hinge: h,
                            });
                        }
                        ("rest", _) => {
                            mod_a.set(ModuleState::new());
                            mod_b.set(ModuleState::new());
                        }
                        ("quit", _) => break,
                        (com, args) => println!("unrecognized command {} {:?}", com, args),
                    }
                } else {
                    println!("ERROR: {:?}", args);
                }
            } else {
                println!("ERROR: EMPTY COMMAND STRING");
            }
        } else {
            println!("ERROR: {:?}", line);
        }
    }
}

fn controller_sim<M:Manager + Send + 'static>(mod_a: ModControl<M>, mod_b: ModControl<M>) {
    let config = ModuleConfig {
        axial_cprad: 100.0,
        hinge_cprad: 100.0,
        body_len: 35.0,
        hinge_offset: 7.5,
        diff_offset: 10.0,
    };
    let mut endpoint = create_control_endpoint(mod_a, mod_b, config, 0.1, Duration::from_secs(1));
    let stdin = io::stdin();
    let handle = stdin.lock();
    for line in handle.lines() {
        if let Ok(line) = line {
            let mut split = line.split_whitespace();
            if let Some(command) = split.next() {
                let args: Result<Vec<f32>, _> = split.map(|x| x.parse::<f32>()).collect();
                if let Ok(args) = args {
                    match (command, args.as_slice()) {
                        ("a", &[a, b, c, d]) => {
                            endpoint.send(ControllerCommand::Motion([a, b, c, d].into()));
                        },
                        ("rest", _) => {
                            endpoint.send(ControllerCommand::Rest);
                        }
                        ("quit", _) => break,
                        (com, args) => println!("unrecognized command {} {:?}", com, args),
                    }
                } else {
                    println!("ERROR: {:?}", args);
                }
            } else {
                println!("ERROR: EMPTY COMMAND STRING");
            }
        } else {
            println!("ERROR: {:?}", line);
        }
    }
}


#[derive(Copy, Clone, Debug)]
struct ArmState {
    pos: Cyl,
}

fn main() {
    let manager = Arc::new(Mutex::new(motor::DirectManager::new(motor::load_config("config.json").unwrap())));
    let mut a = ModControl::new_manager(manager.clone(), ModLayout::new_a());//new_kiss_pair(0.1);
    let mut b = ModControl::new_manager(manager.clone(), ModLayout::new_b());//new_kiss_pair(0.1);
    //controller_sim(a, b);
    command(&mut a, &mut b);
    //control_main();
}

fn control_main() {
    let filenames = vec![
        "/dev/ttyACM0",
        "/dev/ttyACM1",
        "/dev/ttyACM2",
        "/dev/ttyACM3",
    ];
    let mut controls = Vec::new();
    for file in filenames {
        let mut control = Control::open(file).unwrap();
        control.request_id();
        control.drain_serial();
        controls.push(control);
    }
    controls.sort_by_key(|x| x.id.unwrap_or(0));
    for control in &controls {
        println!("{:?}", control.id);
    }
    let c0 = controls.remove(0);
    let c1 = controls.remove(0);
    let c2 = controls.remove(0);
    let c3 = controls.remove(0);
    let mut a = ModControl::new_manager(Arc::new(Mutex::new(())), ModLayout::new_a());//new_kiss_pair(0.1);
    let mut b = ModControl::new_manager(Arc::new(Mutex::new(())), ModLayout::new_b());//new_kiss_pair(0.1);
    command(&mut a, &mut b);
}
fn old_main() {
    let desc = ModDesc::new_default();
    let mut current = ModuleState::new();
    let ret = ModuleState::new();
    let test = ModuleState {
        axial: 0.2,
        near_diff: 0.5,
        far_diff: 0.5,
        hinge: 0.5,
    };
    let mut hinge = Control::open("/dev/ttyACM0").unwrap();
    let mut diff = Control::open("/dev/ttyACM1").unwrap();
    for i in 0..20 {
        thread::sleep(Duration::from_millis(100));
        desc.mod_set(&mut diff, &mut hinge, &current);
        current.approach(&test, 0.05);
    }
    for i in 0..30 {
        thread::sleep(Duration::from_millis(100));
        desc.mod_set(&mut diff, &mut hinge, &current);
        current.approach(&ret, 0.05);
    }
}

fn display_test() {
    let config = ModuleConfig {
        axial_cprad: 100.0,
        hinge_cprad: 100.0,
        body_len: 1.0,
        hinge_offset: 0.25,
        diff_offset: 0.25,
    };
    let mut window = Window::new("Kiss3d: cube");
    let mut c1 = window.add_cube(0.25, 0.25, 0.25);
    let mut c2 = window.add_cube(0.25, 0.25, 0.25);
    let mut c3 = window.add_cube(0.25, 0.25, 0.25);

    c1.set_color(1.0, 0.0, 0.0);
    c2.set_color(1.0, 1.0, 0.0);
    c3.set_color(0.0, 1.0, 1.0);

    window.set_light(Light::StickToCamera);

    let mut t = 0.0;
    while window.render() {
        let pos = Cyl {
            r: 1.0,
            z: t.sin() * 0.4 + 1.5,
            theta: 0.0,
        };
        let stuff = theta_phi_to_diff(t * 0.3, 1.0);
        let (s1, s2) = sym_2mod_a(pos, &config, 0.0).unwrap();
        /*ModuleState {
            hinge: 0.0,
            axial: stuff.2,
            near_diff: stuff.1,
            far_diff: stuff.0,
        };*/
        println!("{:?}, {:?}", s1, s2);
        t += 0.05;
        let t2 = config.diff_to_hinge(&s1);
        window.draw_line(
            &[0.0, 0.0, config.diff_offset].into(),
            &(t2 * Point3::from([0.0, 0.0, -config.hinge_offset])),
            &[1.0, 1.0, 1.0].into(),
        );
        c2.set_local_transformation(t2);
        c3.set_local_transformation(
            t2 * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), consts::PI)
                * config.hinge_to_diff(&s2),
        );
    }
}
