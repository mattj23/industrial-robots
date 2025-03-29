//! Module for FANUC robot products.

use crate::{Iso3, Vector3};

mod crx;

pub use crx::Crx;

/// This is the transformation which rotates the world XYZ coordinate system to the FANUC flange
/// convention where Z is pointing directly out of the flange, Y is inverted from the world Y axis,
/// and X is pointing straight up.
fn end_adjust() -> Iso3 {
    Iso3::rotation(Vector3::new(2.221441469079183, 0.0, 2.221441469079183))
}

/// Convert FANUC joint angles from degrees to radians, including the J2/J3 interaction quirk.
///
/// # Arguments
///
/// * `joints`: a slice of 6 joint angles in degrees as they would be in the FANUC controller
///
/// returns: [f64; 6]
fn joints_to_rad(joints: &[f64]) -> [f64; 6] {
    let mut rad_joints = [0.0; 6];
    for (i, j) in joints.iter().enumerate() {
        rad_joints[i] = j.to_radians();
    }
    rad_joints[2] += rad_joints[1];
    rad_joints
}

/// Convert kinematic joint angles from radians to degrees, including the J2/J3 interaction quirk.
/// The result will be a set of angles in degrees as they would be displayed in the FANUC
/// controller.
///
/// # Arguments
///
/// * `rad_joints`: a slice of 6 joint angles in radians as they would be in the kinematic model
///
/// returns: [f64; 6]
fn rad_to_joints(rad_joints: &[f64]) -> [f64; 6] {
    let mut joints = [0.0; 6];
    for (i, j) in rad_joints.iter().enumerate() {
        joints[i] = j.to_degrees();
    }
    joints[2] -= joints[1];
    joints
}
