use std::f32::consts;

use std::thread;
use std::time::Duration;
use std::path::Path;
use std::rc::Rc;
use std::cell::RefCell;
use std::sync::{Arc, Mutex};

use kiss3d::light::Light;
use kiss3d::window::Window;
use kiss3d::loader;
use nalgebra::*;
use motor::Manager;

use std::sync::mpsc;

use crate::control::*;

pub struct ModControl<M:Manager> {
    manager: Arc<Mutex<M>>,
    layout: ModLayout
}

#[derive(Debug)]
pub struct ModLayout {
    pub hinge: String,
    pub axial: String,
    pub diff_a: String,
    pub diff_b: String,
}

impl ModLayout {
    pub fn new_a() -> Self {
        ModLayout {
            hinge: "A_Hinge".into(),
            axial: "A_Axial".into(),
            diff_a: "A_Diff_Planar".into(),
            diff_b: "A_Diff".into(),
        }
    }
    pub fn new_b() -> Self {
        ModLayout {
            hinge: "B_Hinge".into(),
            axial: "B_Axial".into(),
            diff_a: "B_Diff_Planar".into(),
            diff_b: "B_Diff".into(),
        }
    }
}

impl <M:Manager> ModControl<M> {
    pub fn new_manager(m: Arc<Mutex<M>>, layout: ModLayout) -> Self {
        ModControl { 
            manager: m,
            layout: layout
        }
    }
    pub fn set(&mut self, state: ModuleState) -> () {
        let target = state.nearest_valid();
        let mut dev = self.manager.lock().unwrap();
        dbg!(dev.get_motors());
        dbg!(&self.layout);
        dbg!(&target);
        dbg!(dev.set_motor_rad(&self.layout.hinge, target.hinge));
        dbg!(dev.set_motor_rad(&self.layout.axial, target.axial));
        // near_diff = (diff_a + diff_b) / 2.0
        // far_diff = (diff_a - diff_b) / 2.0
        // diff_a = near_diff + far_diff
        // diff_b = near_diff - far_diff
        dev.set_motor_rad(&self.layout.diff_a, target.near_diff + target.far_diff);
        dev.set_motor_rad(&self.layout.diff_b, target.near_diff - target.far_diff);
    }
}
/*
    pub fn new(mut diff: Control, mut hinge: Control, desc: ModDesc, speed: f32) -> Self {
        let (send, recv) = mpsc::channel::<ModuleState>();
        std::thread::spawn(move || {
            let mut current = ModuleState::new();
            let mut target = ModuleState::new();
            loop {
                thread::sleep(Duration::from_millis(100));
                match recv.try_recv() {
                    Ok(msg) => {
                        target = msg.nearest_valid();
                        println!("{:?}", target);
                        println!("{:?}", current);
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        break;
                    }
                    Err(_) => (),
                }
                current.approach(&target, speed / 10.0);
                desc.mod_set(&mut diff, &mut hinge, &current);
            }
        });
        ModControl { send: send }
    }
    pub fn new_kiss_pair(speed: f32) -> (Self, Self) {
        let (send1, recv1) = mpsc::channel::<ModuleState>();
        let (send2, recv2) = mpsc::channel::<ModuleState>();
        std::thread::spawn(move || {
            let mut current1 = ModuleState::new();
            let mut target1 = ModuleState::new();
            let mut current2 = ModuleState::new();
            let mut target2 = ModuleState::new();
            let config = ModuleConfig {
                axial_cprad: 100.0,
                hinge_cprad: 100.0,
                body_len: 2.8,
                hinge_offset: 0.8,
                diff_offset: 0.5,
            };
            fn matrices(a: &ModuleState, b: &ModuleState, config: &ModuleConfig) -> Vec<Isometry3<f32>> {
                let mut out = vec![Isometry3::identity()];
                let mut current = Isometry3::identity();
                current.append_translation_mut(&Translation3::new(0.0, 0.0, config.diff_offset));
                current.append_rotation_mut(&UnitQuaternion::from_axis_angle(
                        &Vector3::x_axis(),
                        a.far_diff,
                        ));
                out.push(current);
                current *= &UnitQuaternion::from_axis_angle(
                        &Vector3::y_axis(),
                        -a.near_diff,
                        );
                out.push(current);
                current *= &UnitQuaternion::from_axis_angle(
                        &Vector3::z_axis(),
                        a.axial,
                        );
                out.push(current);
                current *= &Translation3::new(0.0, 0.0, config.body_len);
                current *= &UnitQuaternion::from_axis_angle(&Vector3::y_axis(), a.hinge);
                out.push(current);
                current *= &Translation3::new(0.0, 0.0, config.hinge_offset);
                out.push(current);
                current *= &Translation3::new(0.0, 0.0, config.hinge_offset);
                out.push(current);
                current *= &UnitQuaternion::from_axis_angle(&Vector3::y_axis(), b.hinge);
                current *= &Translation3::new(0.0, 0.0, config.body_len);
                out.push(current);
                current *= &UnitQuaternion::from_axis_angle(
                        &Vector3::z_axis(),
                        b.axial,
                        );
                out.push(current);
                current *= &UnitQuaternion::from_axis_angle(
                        &Vector3::y_axis(),
                        -b.near_diff,
                        );
                out.push(current);
                current *= &UnitQuaternion::from_axis_angle(
                        &Vector3::x_axis(),
                        b.far_diff,
                        );
                current *= &Translation3::new(0.0, 0.0, config.diff_offset);
                out.push(current);
                out

            }
            let mut window = Window::new("Kiss3d: cube");
            let mut group = window.add_group();
            let mut c1 = group.add_cube(0.25, 0.25, 0.25);
            let mut c2 = group.add_cube(0.25, 0.25, 0.25);
            let mut c3 = group.add_cube(0.25, 0.25, 0.25);
            group.set_local_transformation(
                Isometry3::identity() *
                &Translation3::new(0.0, -3.0, 0.0) *
                &UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -consts::FRAC_PI_2));
            let mut nodes = Vec::new();
            for i in 0..12 {
                nodes.push(group.add_group());
            }
            let mut models: Vec<_> = vec!["segment_0", "segment_1", "segment_2", "segment_3", "segment_4"].into_iter().map(
                |x| loader::obj::parse_file(&Path::new(&format!("models/{}.obj", x)), &Path::new("models"), "models").unwrap().pop().unwrap()).collect();
            let models: Vec<_> = models.into_iter().map(|x| Rc::new(RefCell::new(x.1))).collect();

            {
                let mut colors = (0..).map(|i| {
                    let t = (i as f32) / 1.5;
                    let m = 0.5;
                    (m*(1.0 + t.cos()), m*(1.0 + (t + 2.0*consts::FRAC_PI_3).cos()), m*(1.0 + (t + 4.0*consts::FRAC_PI_3).cos()))
                });
                let base = Isometry3::identity() * &UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -consts::FRAC_PI_2) * &Translation3::new(0.0, -2.0, 0.0);
                let offsets = vec![
                    (0.0, 0.0),
                    (config.diff_offset, 0.0),
                    (config.diff_offset, 0.0),
                    (config.diff_offset, consts::FRAC_PI_2),
                    (config.diff_offset+config.body_len, consts::FRAC_PI_2),
                    (config.diff_offset+config.body_len, 0.0)];
                for i in 0..models.len() {
                    let mut mesh = nodes[i].add_mesh(models[i].clone(), [0.01, 0.01, 0.01].into());
                    mesh.set_local_transformation(
                        &UnitQuaternion::from_axis_angle(&Vector3::z_axis(), offsets[i].1) *
                        &Translation3::new(0.0, 0.0, -offsets[i].0) *
                        base.clone());
                    let color = colors.next().unwrap();
                    mesh.set_color(color.0, color.1, color.2);
                }
                for i in (0..models.len()).rev() {
                    let mut mesh = nodes[10-i].add_mesh(models[i].clone(), [0.01, 0.01, 0.01].into());
                    mesh.set_local_transformation(
                        &UnitQuaternion::from_axis_angle(&Vector3::x_axis(), consts::PI) *
                        &UnitQuaternion::from_axis_angle(&Vector3::z_axis(), offsets[i].1) *
                        &Translation3::new(0.0, 0.0, -offsets[i].0) *
                        base.clone());
                    let color = colors.next().unwrap();
                    mesh.set_color(color.0, color.1, color.2);
                }
            }

            c1.set_color(1.0, 0.0, 0.0);
            c2.set_color(1.0, 1.0, 0.0);
            c3.set_color(0.0, 1.0, 1.0);

            window.set_light(Light::StickToCamera);

            let mut t = 0.0;
            'renderloop: while window.render() {
                loop {
                match recv1.try_recv() {
                    Ok(msg) => {
                        target1 = msg.nearest_valid();
                        //println!("{:?}", target1);
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        break 'renderloop;
                    }
                    _ => break,
                };
                }
                loop {
                match recv2.try_recv() {
                    Ok(msg) => {
                        target2 = msg.nearest_valid();
                        //println!("{:?}", target2);
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        break 'renderloop;
                    }
                    _ => break,
                };
                }
                current1.approach(&target1, speed / 30.0);
                current2.approach(&target2, speed / 30.0);
                /*ModuleState {
                    hinge: 0.0,
                    axial: stuff.2,
                    near_diff: stuff.1,
                    far_diff: stuff.0,
                };*/
                t += 0.05;
                let t2 = config.diff_to_hinge(&current1);
                /*window.draw_line(
                    &[0.0, 0.0, config.diff_offset].into(),
                    &(t2 * Point3::from([0.0, 0.0, -config.hinge_offset])),
                    &[1.0, 1.0, 1.0].into(),
                );*/
                c2.set_local_transformation(t2);
                c3.set_local_transformation(t2 * config.hinge_to_diff(&current2));
                let mats = matrices(&current1, &current2, &config);
                for i in 0..11 {
                    nodes[i].set_local_transformation(mats[i]);
                }
                //dbg!(c3.local_translation());
            }
        });
        (ModControl { send: send1 }, ModControl { send: send2 })
    }
    pub fn set(&mut self, state: ModuleState) -> () {
        self.send.send(state);
    }
}*/

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ModuleState {
    pub hinge: f32,
    pub axial: f32,
    pub near_diff: f32,
    pub far_diff: f32,
}

impl ModuleState {
    pub fn approach(&mut self, other: &ModuleState, amount: f32) {
        self.hinge += (other.hinge - self.hinge).max(-amount).min(amount);
        self.axial += (other.axial - self.axial).max(-amount).min(amount);
        self.near_diff += (other.near_diff - self.near_diff).max(-amount).min(amount);
        self.far_diff += (other.far_diff - self.far_diff).max(-amount).min(amount);
    }
    pub fn new() -> Self {
        ModuleState {
            hinge: 0.0,
            near_diff: 0.0,
            far_diff: 0.0,
            axial: 0.0,
        }
    }
    pub fn nearest_valid(&self) -> ModuleState {
        let hinge_limit = 90.0 * consts::PI / 180.0;
        let near_limit = 60.0 * consts::PI / 180.0;
        let far_limit = 70.0 * consts::PI / 180.0;
        let axial_limit = 270.0 * consts::PI / 180.0;
        ModuleState {
            hinge: self.hinge.min(hinge_limit).max(-hinge_limit),
            axial: self.axial.min(axial_limit).max(-axial_limit),
            near_diff: self.near_diff.min(near_limit).max(-near_limit),
            far_diff: self.far_diff.min(far_limit).max(-far_limit),
        }
    }
    pub fn is_valid(&self) -> bool {
        &self.nearest_valid() == self
    }
}

pub struct ModuleConfig {
    pub axial_cprad: f32,
    pub hinge_cprad: f32,
    pub body_len: f32,
    pub hinge_offset: f32,
    pub diff_offset: f32,
}

impl ModuleConfig {
    pub fn hinge_to_diff(&self, state: &ModuleState) -> Isometry3<f32> {
        let mut out = Isometry3::identity();
        out.append_translation_mut(&Translation3::new(0.0, 0.0, self.diff_offset));
        out.append_rotation_mut(&UnitQuaternion::from_axis_angle(
            &Vector3::x_axis(),
            state.far_diff,
        ));
        out.append_rotation_mut(&UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            -state.near_diff,
        ));
        out.append_translation_mut(&Translation3::new(0.0, 0.0, self.body_len));
        out.append_rotation_mut(&UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            state.axial,
        ));
        out.append_rotation_mut(
            &UnitQuaternion::from_axis_angle(&Vector3::y_axis(), state.hinge).into(),
        );
        out.append_translation_mut(&Translation3::new(0.0, 0.0, self.hinge_offset));
        out
    }
    pub fn diff_to_hinge(&self, state: &ModuleState) -> Isometry3<f32> {
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), consts::PI)
            * self.hinge_to_diff(state).inverse()
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), consts::PI)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ModDesc {
    hinge_cprad: f32,
    axial_cprad: f32,
    axial_is_a: bool,
    coplanar_is_a: bool,
}

impl ModDesc {
    pub fn state_to_diff_params(&self, state: &ModuleState) -> (f32, f32) {
        let a = self.hinge_cprad * (state.near_diff + state.far_diff) / 2.0;
        let b = self.hinge_cprad * (-state.near_diff + state.far_diff) / 2.0;
        if self.coplanar_is_a {
            (a, b)
        } else {
            (b, a)
        }
    }
    pub fn state_to_hinge_params(&self, state: &ModuleState) -> (f32, f32) {
        let a = self.axial_cprad * state.axial;
        let b = self.hinge_cprad * state.hinge;
        if self.axial_is_a {
            (a, b)
        } else {
            (b, a)
        }
    }
    pub fn mod_set(&self, diff: &mut Control, hinge: &mut Control, state: &ModuleState) -> () {
        let (da, db) = self.state_to_diff_params(state);
        let (ha, hb) = self.state_to_hinge_params(state);
        diff.set(da as i32, db as i32);
        hinge.set(ha as i32, hb as i32);
    }
    pub fn new_default() -> Self {
        ModDesc {
            hinge_cprad: 183.3465,
            axial_cprad: 386.7465,
            axial_is_a: false,
            coplanar_is_a: true,
        }
    }
    pub fn new_default_b() -> Self {
        ModDesc {
            hinge_cprad: 183.3465,
            axial_cprad: 386.7465 * 8.0 / 3.0,
            axial_is_a: false,
            coplanar_is_a: true,
        }
    }
}
