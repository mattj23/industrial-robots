//! Module for the CRX series of collaborative robots. These are non-spherical wrist robots with
//! three parallel axes. The entire series (as of Q1 2025) has the same kinematic structure and
//! differs only in the lengths of the different links.
//!
//! The CRX series also has the linked J2/J3 quirk common to the FANUC robots.  This means that
//! when J2 actuates in the positive direction, J3 actuates the same amount in order to keep the
//! forearm parallel to the robot base.  This means that to use any kinematics model for robots in
//! this series, the J2/J3 angles must be modified on their way in and out.

use crate::fanuc::{end_adjust, joints_to_rad};
use crate::nalgebra::{Translation, UnitQuaternion};
use crate::type_aliases::Frame3;
use crate::Vector3;
use ik_geo::robot::IKSolver;

pub struct Crx {
    z1: f64,
    x1: f64,
    x2: f64,
    y1: f64,
    h: [Vector3; 6],
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
        // The h vectors are the directions of the rotation axes associated with each joint.
        let h = [
            Vector3::z(),
            Vector3::y(),
            -Vector3::y(),
            -Vector3::x(),
            -Vector3::y(),
            -Vector3::x(),
        ];

        Self { z1, x1, x2, y1, h }
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
    pub fn fk(&self, joints: &[f64; 6]) -> Frame3 {
        self.fk_all(joints)[5]
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
    pub fn fk_all(&self, joints: &[f64; 6]) -> [Frame3; 6] {
        let joints = joints_to_rad(joints);

        // The first link is at the origin, rotated by the first joint angle
        let f1 = Frame3::rotation(self.h[0] * joints[0]);

        // J1->J2 has no origin shift
        let f2 = f1 * Frame3::rotation(self.h[1] * joints[1]);

        // J2->J3 shifts up by the z1 value
        let f3 = f2 * Frame3::from_parts(
            Translation::<f64, 3>::new(0.0, 0.0, self.z1),
            UnitQuaternion::new(self.h[2] * joints[2]),
        );

        // J3->J4 has no origin shift
        let f4 = f3 * Frame3::rotation(self.h[3] * joints[3]);

        // J4->J5 shifts by x1, -y1
        let f5 = f4 * Frame3::from_parts(
            Translation::<f64, 3>::new(self.x1, -self.y1, 0.0),
            UnitQuaternion::new(self.h[4] * joints[4]),
        );

        // J5->J6 shifts by x2, then gets re-oriented by the FANUC end
        // effector adjustment
        // let f6 = fk_result(&self.robot, &joints) * end_adjust();
        let f6 = f5 * Frame3::from_parts(
            Translation::<f64, 3>::new(self.x2, 0.0, 0.0),
            UnitQuaternion::new(self.h[5] * joints[5]),
        ) * end_adjust();

        [f1, f2, f3, f4, f5, f6]
    }

    // pub fn ik(&self, target: &Frame3) {
    //     let fk0 = fk_result(&self.robot, &[0.0; 6]);
    //     println!("Reference: {:?}", fk0);
    //
    //     // Undo the end effector adjustment
    //     let target =  target * end_adjust().inverse();
    //
    //     let (r, t) = iso_to_parts(&target);
    //     println!("Target: {:?}", target);
    //     println!("---");
    //     println!("Rotation: {:?}", r);
    //     println!("Translation: {:?}", t);
    //     let solutions = self.robot.ik(r, t);
    //
    //     println!("Solutions: {:?}", solutions);
    //
    //     for (q, is_ls) in solutions {
    //         if is_ls {
    //             println!("LS solution: {:?}", q);
    //         }
    //         else {
    //             println!("Non-LS solution: {:?}", q);
    //         }
    //
    //     }
    // }
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
        let fwd = robot.fk(&j);

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
        let fwd = robot.fk(&j);

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
        let fwd = robot.fk(&j);

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
        let fwd = robot.fk(&j);

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
        let fwd = robot.fk(&j);

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
        let fwd = robot.fk(&j);

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
        let fwd = robot.fk(&j);

        assert_relative_eq!(expected, fwd, epsilon = 1e-6);
        Ok(())
    }

    #[test]
    fn crx5ia_bulk() -> Result<()> {
        let bytes = include_bytes!("test_data/fanuc_crx_5ia.json");
        let data: Vec<([f64; 6], [f64; 16])> = serde_json::from_slice(bytes)?;

        let robot = Crx::new_5ia();
        for (joints, expected) in data {
            let fwd = robot.fk(&joints);
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
            let fwd = robot.fk(&joints);
            let expected = row_slice_to_iso(&expected)?;
            assert_relative_eq!(fwd, expected, epsilon = 1e-6);
        }

        Ok(())
    }
}
