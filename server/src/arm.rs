use crate::module::*;

use std::f32::consts;

#[derive(Copy, Clone, Debug)]
pub struct Cyl {
    pub z: f32,
    pub r: f32,
    pub theta: f32,
}

/// return order: (far_diff, near_diff, axial)
pub fn theta_phi_to_diff(theta: f32, phi: f32) -> (f32, f32, f32) {
    //let theta = theta + consts::FRAC_PI_2;
    let near = (phi.sin() * theta.cos()).asin();
    let far = (phi.sin() * theta.sin() / near.cos()).asin();
    let axial_y = theta.sin() * phi.sin()/near.cos();
    let axial_x = theta.cos() / near.cos();
    //let axial = (-axial_x).atan2(-axial_y);
    let axial = theta + ((theta.sin()*theta.cos()*(1.0 - phi.cos()))/near.cos()).asin();
    (far, near, axial)
}

pub fn sym_2mod_a(mut pos: Cyl, config: &ModuleConfig, phi: f32) -> Option<(ModuleState, ModuleState)> {
    pos.z -= config.diff_offset;
    // phi == 0 means that the end effector is pointing in the same direction as the r-axis
    pos.z -= phi.sin() * config.diff_offset;
    pos.r -= phi.cos() * config.diff_offset;
    let half_elbow_angle = {
        let len = (pos.r * pos.r + pos.z * pos.z).sqrt();
        let newlen = len - config.hinge_offset * 2.0;
        if newlen < 0.0 {
            return None;
        }
        pos.z = pos.z * newlen / len;
        pos.r = pos.r * newlen / len;
        let elbow_offset = config.body_len * config.body_len - newlen * newlen / 4.0;
        if elbow_offset < 0.0 {
            return None;
        }
        let elbow_offset = elbow_offset.sqrt();
        let elbow_angle = (newlen / 2.0).atan2(elbow_offset);
        elbow_angle
    };
    let dest_phi = pos.r.atan2(pos.z); // angle from the z-axis to the target
    let mod_b = ModuleState {
        hinge: -consts::FRAC_PI_2 + half_elbow_angle,
        axial: 0.0,
        near_diff: half_elbow_angle + phi - dest_phi,
        far_diff: 0.0,
    };
    let base_params = theta_phi_to_diff(pos.theta, dest_phi + mod_b.hinge);
    let mod_a = ModuleState {
        hinge: mod_b.hinge,
        axial: base_params.2,
        near_diff: base_params.1,
        far_diff: base_params.0,
        /*
        axial: 0.0,
        near_diff: dest_phi + mod_b.hinge,
        far_diff: 0.0,*/
    };
    //println!("{:?}", mod_a);

    Some((mod_a, mod_b))
}
