//! Module for FANUC robot products.

use crate::{Frame3, Point3, Vector3};

mod crx;

use crate::micro_mesh::bytes_to_mesh;
pub use crx::Crx;

#[cfg(feature = "mesh_fanuc_crx5ia")]
pub fn crx5ia_mesh() -> Vec<(Vec<Point3>, Vec<[u32; 3]>)> {
    vec![
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-5ia-j0.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-5ia-j1.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-5ia-j2.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-5ia-j3.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-5ia-j4.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-5ia-j5.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-5ia-j6.smol")).unwrap(),
    ]
}

#[cfg(feature = "mesh_fanuc_crx10ia")]
pub fn crx10ia_mesh() -> Vec<(Vec<Point3>, Vec<[u32; 3]>)> {
    vec![
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-10ia-j0.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-10ia-j1.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-10ia-j2.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-10ia-j3.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-10ia-j4.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-10ia-j5.smol")).unwrap(),
        bytes_to_mesh(include_bytes!("./fanuc/meshes/crx-10ia-j6.smol")).unwrap(),
    ]
}

/// This is the transformation which rotates the world XYZ coordinate system to the FANUC flange
/// convention where Z is pointing directly out of the flange, Y is inverted from the world Y axis,
/// and X is pointing straight up.
fn end_adjust() -> Frame3 {
    Frame3::rotation(Vector3::new(2.221441469079183, 0.0, 2.221441469079183))
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
