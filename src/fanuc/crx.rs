//! Module for the CRX series of collaborative robots. These are non-spherical wrist robots with
//! three parallel axes. The entire series (as of Q1 2025) has the same kinematic structure and
//! differs only in the lengths of the different links.
//!
//! The CRX series also has the linked J2/J3 quirk common to the FANUC robots.  This means that
//! when J2 actuates in the positive direction, J3 actuates the same amount in order to keep the
//! forearm parallel to the robot base.  This means that to use any kinematics model for robots in
//! this series, the J2/J3 angles must be modified on their way in and out.

use crate::fanuc::{end_adjust, joints_to_rad};
use crate::helpers::{fk_result, iso_to_parts, parts_to_iso};
use crate::nalgebra::{Translation, UnitQuaternion};
use crate::type_aliases::Frame3;
use crate::{Point3, Vector3};
use ik_geo::inverse_kinematics::auxiliary::Matrix3x7;
use ik_geo::nalgebra::Matrix3x6;
use ik_geo::robot::{Robot, three_parallel, IKSolver, three_parallel_two_intersecting, two_intersecting, two_parallel};

pub struct Crx {
    robot: Robot,
    pub p_vectors: [Vector3; 7],
    h_vectors: [Vector3; 6],
}

impl Crx {
    /// Internal constructor for the CRX series of robots.
    ///
    /// # Arguments
    ///
    /// * `z1`: The height from the J2 axis to the J3 axis (410mm on the CRX-5iA datasheet).
    /// * `x1`: The length from the J3 axis to the J5 axis (430mm on the CRX-5iA datasheet).
    /// * `x2`: The length from the J5 axis to the robot flange (145mm on the CRX-5iA datasheet).
    /// * `y1`: The offset from the J1 axis to the J2 axis (130mm on the CRX-5iA datasheet).
    ///
    /// returns: Crx
    fn new(z1: f64, x1: f64, x2: f64, y1: f64) -> Self {
        let p = crx_p_matrix(z1, x1, x2, y1);
        let h = crx_h_matrix();

        let mut p_vectors = [Vector3::zeros(); 7];
        let mut h_vectors = [Vector3::zeros(); 6];

        for (i, col) in p.column_iter().enumerate() {
            p_vectors[i] = Vector3::new(col[0], col[1], col[2]);
        }

        for (i, col) in h.column_iter().enumerate() {
            h_vectors[i] = Vector3::new(col[0], col[1], col[2]);
        }

        Self {
            robot: two_parallel(h, p),
            p_vectors,
            h_vectors,
        }
    }

    /// Creates a new CRX-5iA robot
    pub fn new_5ia() -> Self {
        Self::new(410.0, 430.0, 145.0, 130.0)
    }

    pub fn new_10ia() -> Self {
        Self::new(540.0, 540.0, 160.0, 150.0)
    }

    /// Compute the forward kinematics of a series of joint angles for the CRX series of robots.
    /// The joints should be provided in degrees as they would appear in the robot controller. The
    /// output will be a `Frame3` object representing the position and orientation of the robot's
    /// flange in relation to the robot origin.
    ///
    /// The output frame will match the FANUC controller in position and orientation.
    ///
    /// # Arguments
    ///
    /// * `joints`: The joint angles for the robot in degrees. This should be an array of 6 values
    ///   representing the angles for each joint in the order of J1, J2, J3, J4, J5, and J6.
    ///
    /// returns: Isometry<f64, Unit<Quaternion<f64>>, 3>
    pub fn forward(&self, joints: &[f64; 6]) -> Frame3 {
        let joints = joints_to_rad(joints);
        fk_result(&self.robot, &joints) * end_adjust()
    }

    /// Compute the forward kinematics of a series of joint angles for the CRX series of robots,
    /// returning the full kinematic chain for each joint in the robot. This will return an array
    /// of `Frame3` objects representing the position and orientation of each joint in relation
    /// to the robot origin. This can be useful for visualizing the full kinematic chain of the
    /// robot and understanding how each joint contributes to the overall position and
    /// orientation of the robot's flange.
    ///
    /// The final frame in the array will represent the position and orientation of the robot's
    /// flange, and will be identical to the result of the `forward` method, matching the expected
    /// value of the actual robot controller. The other frames in the array will be at the
    /// kinematic link origins, and do not have any corresponding values in the actual robot.
    ///
    /// # Arguments
    ///
    /// * `joints`: The joint angles for the robot in degrees. This should be an array of 6 values
    ///  representing the angles for each joint in the order of J1, J2, J3, J4, J5, and J6.
    ///
    /// returns: [Isometry<f64, Unit<Quaternion<f64>>, 3>; 6]
    pub fn forward_with_links(&self, joints: &[f64; 6]) -> [Frame3; 6] {
        let joints = joints_to_rad(joints);

        // The first link is at the origin, rotated by the first joint angle
        let f1 = Frame3::rotation(self.h_vectors[0] * joints[0]);

        let f2 = f1 * self.local_link_frame(1, joints[1]);
        let f3 = f2 * self.local_link_frame(2, joints[2]);
        let f4 = f3 * self.local_link_frame(3, joints[3]);
        let f5 = f4 * self.local_link_frame(4, joints[4]);
        let f6 = fk_result(&self.robot, &joints) * end_adjust();

        [f1, f2, f3, f4, f5, f6]
    }

    pub fn ik(&self, target: &Frame3) {
        let fk0 = fk_result(&self.robot, &[0.0; 6]);
        println!("Reference: {:?}", fk0);

        // Undo the end effector adjustment
        let target =  target * end_adjust().inverse();

        let (r, t) = iso_to_parts(&target);
        println!("Target: {:?}", target);
        println!("---");
        println!("Rotation: {:?}", r);
        println!("Translation: {:?}", t);
        let solutions = self.robot.ik(r, t);

        println!("Solutions: {:?}", solutions);

        for (q, is_ls) in solutions {
            if is_ls {
                println!("LS solution: {:?}", q);
            }
            else {
                println!("Non-LS solution: {:?}", q);
            }

        }
    }

    fn local_link_frame(&self, i: usize, joint: f64) -> Frame3 {
        Frame3::from_parts(
            Translation::<f64, 3>::new(
                self.p_vectors[i].x,
                self.p_vectors[i].y,
                self.p_vectors[i].z,
            ),
            UnitQuaternion::new(self.h_vectors[i] * joint),
        )
    }
}

/// Creates the rotation axes matrix for the CRX series of robots. These are the same across models
/// because the robots are basically identical except for the lengths of the links.
fn crx_h_matrix() -> Matrix3x6<f64> {
    Matrix3x6::<f64>::new(
        0.0, 0.0, 0.0, -1.0, 0.0, -1.0, 0.0, 1.0, -1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0,
    )
}

/// Create the p matrix for the CRX series of robots. These are the 3D vectors from the origin of
/// each link to the origin of the next link in the IK geo model (see pages 5 and 6 of the IK-Geo
/// paper).
///
/// The FANUC CRX series has five unique link lengths which distinguish the kinematics of each
/// model, and these must be provided as the parameters to this function.
///
/// # Arguments
///
/// * `z1`: The height from the J2 axis to the J3 axis (410mm on the CRX-5iA datasheet).
/// * `x1`: The length from the J3 axis to the J5 axis (430mm on the CRX-5iA datasheet).
/// * `x2`: The length from the J5 axis to the robot flange (145mm on the CRX-5iA datasheet).
/// * `y1`: The offset from the J1 axis to the J2 axis (130mm on the CRX-5iA datasheet).
///
/// returns: Matrix<f64, Const<3>, Const<7>, ArrayStorage<f64, 3, 7>>
fn crx_p_matrix(z1: f64, x1: f64, x2: f64, y1: f64) -> Matrix3x7<f64> {
    let mut p = Matrix3x7::<f64>::zeros();
    p[(2, 2)] = z1;
    p[(0, 4)] = x1;
    p[(1, 4)] = -y1;
    p[(0, 5)] = x2;
    p
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::row_slice_to_iso;
    use crate::{Point3, Result};
    use approx::assert_relative_eq;

    #[test]
    fn zero_position() -> Result<()> {
        let j = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let expected = row_slice_to_iso(&[
            0.0, 0.0, 1.0, 575.0, 0.0, -1.0, 0.0, -130.0, 1.0, 0.0, 0.0, 410.0, 0.0, 0.0, 0.0, 1.0,
        ])?;
        let robot = Crx::new_5ia();
        let fwd = robot.forward(&j);

        assert_relative_eq!(expected, fwd, epsilon = 1e-6);
        Ok(())
    }

    #[test]
    fn only_j1() -> Result<()> {
        let j = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let expected = row_slice_to_iso(&[
            0.0,
            0.1736481776669304,
            0.984807753012208,
            588.8387210787206,
            0.0,
            -0.984807753012208,
            0.1736481776669304,
            -28.17730573310209,
            1.0,
            0.0,
            0.0,
            410.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ])?;
        let robot = Crx::new_5ia();
        let fwd = robot.forward(&j);

        assert_relative_eq!(expected, fwd, epsilon = 1e-6);
        Ok(())
    }

    #[test]
    fn only_j2() -> Result<()> {
        let j = [0.0, 10.0, 0.0, 0.0, 0.0, 0.0];
        let expected = row_slice_to_iso(&[
            0.0,
            0.0,
            1.0,
            646.1957528434414,
            0.0,
            -1.0,
            0.0,
            -130.0,
            1.0,
            0.0,
            0.0,
            403.7711787350052,
            0.0,
            0.0,
            0.0,
            1.0,
        ])?;
        let robot = Crx::new_5ia();
        let fwd = robot.forward(&j);

        assert_relative_eq!(expected, fwd, epsilon = 1e-6);
        Ok(())
    }

    #[test]
    fn only_j3() -> Result<()> {
        let j = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0];
        let expected = row_slice_to_iso(&[
            -0.17364817766693028,
            0.0,
            0.984807753012208,
            566.2644579820196,
            0.0,
            -1.0,
            0.0,
            -130.0,
            0.984807753012208,
            0.0,
            0.17364817766693028,
            509.84770215848494,
            0.0,
            0.0,
            0.0,
            1.0,
        ])?;
        let robot = Crx::new_5ia();
        let fwd = robot.forward(&j);

        assert_relative_eq!(expected, fwd, epsilon = 1e-6);
        Ok(())
    }

    #[test]
    fn only_j4() -> Result<()> {
        let j = [0.0, 0.0, 0.0, 10.0, 0.0, 0.0];
        let expected = row_slice_to_iso(&[
            0.0,
            0.0,
            1.0,
            575.0,
            0.17364817766693028,
            -0.984807753012208,
            0.0,
            -128.02500789158705,
            0.984807753012208,
            0.17364817766693028,
            0.0,
            432.5742630967009,
            0.0,
            0.0,
            0.0,
            1.0,
        ])?;
        let robot = Crx::new_5ia();
        let fwd = robot.forward(&j);

        assert_relative_eq!(expected, fwd, epsilon = 1e-6);
        Ok(())
    }

    #[test]
    fn only_j5() -> Result<()> {
        let j = [0.0, 0.0, 0.0, 0.0, 10.0, 0.0];
        let expected = row_slice_to_iso(&[
            -0.17364817766693028,
            0.0,
            0.984807753012208,
            572.7971241867701,
            0.0,
            -1.0,
            0.0,
            -130.0,
            0.984807753012208,
            0.0,
            0.17364817766693028,
            435.1789857617049,
            0.0,
            0.0,
            0.0,
            1.0,
        ])?;
        let robot = Crx::new_5ia();
        let fwd = robot.forward(&j);

        assert_relative_eq!(expected, fwd, epsilon = 1e-6);
        Ok(())
    }

    #[test]
    fn only_j6() -> Result<()> {
        let j = [0.0, 0.0, 0.0, 0.0, 0.0, 10.0];
        let expected = row_slice_to_iso(&[
            0.0,
            0.0,
            1.0,
            575.0,
            0.17364817766693028,
            -0.984807753012208,
            0.0,
            -130.0,
            0.984807753012208,
            0.17364817766693028,
            0.0,
            410.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ])?;
        let robot = Crx::new_5ia();
        let fwd = robot.forward(&j);

        assert_relative_eq!(expected, fwd, epsilon = 1e-6);
        Ok(())
    }

    #[test]
    fn crx5ia_bulk() -> Result<()> {
        let bytes = include_bytes!("test_data/fanuc_crx_5ia.json");
        let data: Vec<([f64; 6], [f64; 16])> = serde_json::from_slice(bytes)?;

        let robot = Crx::new_5ia();
        for (joints, expected) in data {
            let fwd = robot.forward(&joints);
            let expected = row_slice_to_iso(&expected)?;
            assert_relative_eq!(fwd, expected, epsilon = 1e-6);
        }

        Ok(())
    }

    #[test]
    fn crx10ia_bulk() -> Result<()> {
        let bytes = include_bytes!("test_data/fanuc_crx_10ia.json");
        let data: Vec<([f64; 6], [f64; 16])> = serde_json::from_slice(bytes)?;

        let robot = Crx::new_10ia();
        for (joints, expected) in data {
            let fwd = robot.forward(&joints);
            let expected = row_slice_to_iso(&expected)?;
            assert_relative_eq!(fwd, expected, epsilon = 1e-6);
        }

        Ok(())
    }
}
