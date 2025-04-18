//! Module for the CRX series of collaborative robots. These are non-spherical wrist robots with
//! three parallel axes. The entire series (as of Q1 2025) has the same kinematic structure and
//! differs only in the lengths of the different links.
//!
//! The CRX series also has the linked J2/J3 quirk common to the FANUC robots.  This means that
//! when J2 actuates in the positive direction, J3 actuates the same amount in order to keep the
//! forearm parallel to the robot base.  This means that to use any kinematics model for robots in
//! this series, the J2/J3 angles must be modified on their way in and out.

use crate::fanuc::{end_adjust, joints_to_rad, rad_to_joints};
use crate::nalgebra::{Translation, UnitQuaternion};
use crate::type_aliases::Frame3;
use crate::{Point3, Vector3};

pub struct Crx {
    z0: f64,
    z1: f64,
    x1: f64,
    x2: f64,
    y1: f64,
    h: [Vector3; 6],
    ik_d: f64,
}

impl Crx {
    /// The height from the bottom of the mounting flange to the world origin. This doesn't
    /// affect the kinematics, but is a necessary value from the datasheet for doing layout and
    /// position studies.
    pub fn z0(&self) -> f64 {
        self.z0
    }

    /// The height from the J2 axis to the J3 axis
    pub fn z1(&self) -> f64 {
        self.z1
    }

    /// The length from the J3 axis to the J5 axis
    pub fn x1(&self) -> f64 {
        self.x1
    }

    /// The length from the J5 axis to the robot flange
    pub fn x2(&self) -> f64 {
        self.x2
    }

    /// The offset from the J1 axis to the J2 axis
    pub fn y1(&self) -> f64 {
        self.y1
    }

    /// Internal constructor for the CRX series of robots.
    ///
    /// # Arguments
    ///
    /// * `z0`: The height from the bottom of the mounting flange to the world origin. This doesn't
    ///   affect the kinematics, but is a necessary value for doing layout and position studies.
    /// * `z1`: The height from the J2 axis to the J3 axis (410mm on the CRX-5iA datasheet).
    /// * `x1`: The length from the J3 axis to the J5 axis (430mm on the CRX-5iA datasheet).
    /// * `x2`: The length from the J5 axis to the robot flange (145mm on the CRX-5iA datasheet).
    /// * `y1`: The offset from the J1 axis to the J2 axis (130mm on the CRX-5iA datasheet).
    ///
    /// returns: Crx
    fn new(z0: f64, z1: f64, x1: f64, x2: f64, y1: f64) -> Self {
        // The h vectors are the directions of the rotation axes associated with each joint.
        let h = [
            Vector3::z(),
            Vector3::y(),
            -Vector3::y(),
            -Vector3::x(),
            -Vector3::y(),
            -Vector3::x(),
        ];

        let ik_d = (y1.powi(2) + x1.powi(2)).sqrt();

        Self {
            z0,
            z1,
            x1,
            x2,
            y1,
            h,
            ik_d,
        }
    }

    /// Creates a new CRX-5iA robot
    pub fn new_5ia() -> Self {
        Self::new(185.0, 410.0, 430.0, 145.0, 130.0)
    }

    pub fn new_10ia() -> Self {
        Self::new(245.0, 540.0, 540.0, 160.0, 150.0)
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
        let f3 = f2
            * Frame3::from_parts(
                Translation::<f64, 3>::new(0.0, 0.0, self.z1),
                UnitQuaternion::new(self.h[2] * joints[2]),
            );

        // J3->J4 has no origin shift
        let f4 = f3 * Frame3::rotation(self.h[3] * joints[3]);

        // J4->J5 shifts by x1, -y1
        let f5 = f4
            * Frame3::from_parts(
                Translation::<f64, 3>::new(self.x1, -self.y1, 0.0),
                UnitQuaternion::new(self.h[4] * joints[4]),
            );

        // J5->J6 shifts by x2, then gets re-oriented by the FANUC end
        // effector adjustment
        // let f6 = fk_result(&self.robot, &joints) * end_adjust();
        let f6 =
            f5 * Frame3::from_parts(
                Translation::<f64, 3>::new(self.x2, 0.0, 0.0),
                UnitQuaternion::new(self.h[5] * joints[5]),
            ) * end_adjust();

        [f1, f2, f3, f4, f5, f6]
    }

    ///
    ///
    /// # Arguments
    ///
    /// * `d`:
    ///
    /// returns: Option<(f64, f64)>
    ///
    /// # Examples
    ///
    /// ```
    ///
    /// ```
    pub fn ik(&self, target: &Frame3) -> Vec<[f64; 6]> {
        let mut results = Vec::new();
        let (upper_results, lower_results) = self.brute_force_o3(target);

        for theta in upper_results.iter() {
            let (o4, o3s) = self.get_candidates(*theta, target);
            if let Some((u, _)) = o3s {
                results.push(self.calculate_joint_radians(&u, &o4, target));
            }
        }

        for theta in lower_results.iter() {
            let (o4, o3s) = self.get_candidates(*theta, target);
            if let Some((_, l)) = o3s {
                results.push(self.calculate_joint_radians(&l, &o4, target));
            }
        }

        results.iter().map(|r| rad_to_joints(r)).collect()
    }

    fn calculate_joint_radians(&self, o3: &Point3, o4: &Point3, target: &Frame3) -> [f64; 6] {
        let mut joints = [0.0; 6];
        let o5 = target * Point3::new(0.0, 0.0, -self.x2);
        let o6 = target * Point3::new(0.0, 0.0, 0.0);

        // Calculate J1 and the frame which brings O3 into the X-Z plane
        joints[0] = o4.y.atan2(o4.x);
        let f1 = Frame3::rotation(Vector3::z() * joints[0]);
        let o3_j1 = f1.inverse() * o3;

        // Calculate J2
        joints[1] = o3_j1.x.atan2(o3_j1.z);
        let f3 = f1
            * Frame3::translation(o3_j1.x, o3_j1.y, o3_j1.z)
            * Frame3::rotation(Vector3::y() * joints[1]);
        let o4_j3 = f3.inverse() * o4;

        // Calculate J3
        joints[2] = o4_j3.z.atan2(o4_j3.x);
        let f4 = f3
            * Frame3::translation(o4_j3.x, o4_j3.y, o4_j3.z)
            * Frame3::rotation(-Vector3::y() * joints[2]);
        let o5_j4 = f4.inverse() * o5;

        // Calculate J4
        joints[3] = o5_j4.z.atan2(-o5_j4.y);
        let f5 = f4
            * Frame3::translation(o5_j4.x, o5_j4.y, o5_j4.z)
            * Frame3::rotation(-Vector3::x() * joints[3]);
        let o6_j5 = f5.inverse() * o6;

        // Calculate J5
        joints[4] = o6_j5.z.atan2(o6_j5.x);
        let f6 = f5
            * Frame3::translation(o6_j5.x, o6_j5.y, o6_j5.z)
            * Frame3::rotation(-Vector3::y() * joints[4])
            * end_adjust();
        let o6_ztest = f6.inverse() * target * Point3::new(1.0, 0.0, 0.0);

        // Calculate J6
        joints[5] = -o6_ztest.y.atan2(o6_ztest.x);

        joints
    }

    pub fn o3_points(&self, target: &Frame3) -> (Vec<Point3>, Vec<Point3>) {
        // Now we find the corresponding O3 points for each candidate O4 point
        let mut uppers = Vec::new();
        let mut lowers = Vec::new();

        for o4 in self.o4_circle(target).iter() {
            if let Some((u, l)) = self.candidate_o3s(o4) {
                uppers.push(u);
                lowers.push(l);
            }
        }

        (uppers, lowers)
    }

    pub fn o4_circle(&self, target: &Frame3) -> Vec<Point3> {
        // Now we find the candidate o4 points
        let mut o4c = Vec::new();
        let n = 500;
        let step = 2.0 * std::f64::consts::PI / n as f64;

        for i in 0..n {
            let theta = i as f64 * step;
            o4c.push(target * Point3::new(self.y1 * theta.cos(), self.y1 * theta.sin(), -self.x2));
        }

        o4c
    }

    /// Finds the a, h values for a candidate O4 position, where `d` is the distance
    /// between the robot origin and the O4 point. If d is too small or too large for
    /// a valid solution, None is returned.
    ///
    /// The value of `a` is the distance to the corresponding O3 candidate point along
    /// the ray from the candidate O4 point to the origin. The value of `a` is the same
    /// for both the upper and lower associated O3 candidate points.
    ///
    /// The value of `h` is the distance to the corresponding O3 candidate points above
    /// or below the ray from the candidate O4 point to the origin. The upper candidate
    /// O3 point is at (a, h), and the lower candidate O3 point is at (a, -h).
    ///
    /// # Arguments
    ///
    /// * `d`: the distance between the robot origin and the O4 point
    fn corner_xy(&self, d: f64) -> Option<(f64, f64)> {
        // d must be at least as large as the distance between the smaller radius and the
        // larger radius, but no larger than the sum of the two radii
        if d < self.z1.max(self.x1) - self.z1.min(self.x1) || d > self.z1 + self.x1 {
            None
        } else {
            let a = (d.powi(2) + self.x1.powi(2) - self.z1.powi(2)) / (2.0 * d);
            let h = (self.x1.powi(2) - a.powi(2)).sqrt();
            Some((a, h))
        }
    }

    fn refine_theta(&self, t0: f64, e0: f64, t1: f64, e1: f64, target: &Frame3, o5: &Point3) -> f64 {
        // println!("t0: {}, e0: {}", t0, e0);
        // println!("t1: {}, e1: {}", t1, e1);
        
        let f = (0.0 - e0) / (e1 - e0);
        let t = f * (t1 - t0) + t0;
        // println!("f: {}", f);
        // println!("t: {}", t);
        
        t
    }
    
    pub fn brute_force_o3(&self, target: &Frame3) -> (Vec<f64>, Vec<f64>) {
        let n = 1500;
        let o5 = target * Point3::new(0.0, 0.0, -self.x2);
        let step = 2.0 * std::f64::consts::PI / n as f64;

        // There are two places where we may find zeros: (a) crossings, and (b) local minima/maxima.
        // Local minima/maxima will happen where the derivative is zero, while crossings will
        // the last value and the next value have different signs.
        let mut upper_results = Vec::new();
        let mut lower_results = Vec::new();

        let mut last_theta = 0.0;
        let (mut last_eu, mut last_el) = self.error(0.0, target, &o5);
        for i in 0..(n + 1) {
            let theta = (i + 1) as f64 * step;
            let (eu, el) = self.error(theta, target, &o5);

            if !eu.is_nan() && !last_eu.is_nan() {
                if eu.signum() != last_eu.signum() {
                    upper_results.push(self.refine_theta(last_theta, last_eu, theta, eu, target, &o5));
                }

                // We'll do minima/maxima later
            }

            if !el.is_nan() && !last_el.is_nan() {
                if el.signum() != last_el.signum() {
                    lower_results.push(self.refine_theta(last_theta, last_el, theta, el, target, &o5));
                }

                // We'll do minima/maxima later
            }

            last_el = el;
            last_eu = eu;
            last_theta = theta;
        }

        (upper_results, lower_results)
    }

    fn get_candidates(&self, theta: f64, target: &Frame3) -> (Point3, Option<(Point3, Point3)>) {
        let o4 = target * Point3::new(self.y1 * theta.cos(), self.y1 * theta.sin(), -self.x2);
        let v0 = -o4.coords;

        // Vector from the candidate point's projection on the XY plane to the origin
        let vp = Vector3::new(v0.x, v0.y, 0.0);

        // If the vector from the candidate point projection to the origin is zero
        // length, it means that the candidate point is directly above the robot origin,
        // and there's probably some sort of special case
        let e0 = v0.normalize();
        let e2 = v0.cross(&vp).normalize();
        let e1 = e2.cross(&e0).normalize();

        if let Some((a, h)) = self.corner_xy(v0.norm()) {
            let u = e0 * a + e1 * h;
            let l = e0 * a - e1 * h;
            (o4, Some((o4 + u, o4 + l)))
        } else {
            (o4, None)
        }
    }

    pub fn error(&self, theta: f64, target: &Frame3, o5: &Point3) -> (f64, f64) {
        // let o4 = target * Point3::new(self.y1 * theta.cos(), self.y1 * theta.sin(), -self.x2);
        // let v0 = -o4.coords;
        // 
        // // Vector from the candidate point's projection on the XY plane to the origin
        // let vp = Vector3::new(v0.x, v0.y, 0.0);
        // 
        // // If the vector from the candidate point projection to the origin is zero
        // // length, it means that the candidate point is directly above the robot origin,
        // // and there's probably some sort of special case
        // let e0 = v0.normalize();
        // let e2 = v0.cross(&vp).normalize();
        // let e1 = e2.cross(&e0).normalize();
        
        let (_, o3) = self.get_candidates(theta, target);

        if let Some((up, lp)) = o3 {
            let eu = (o5 - up).norm() - self.ik_d;
            let el = (o5 - lp).norm() - self.ik_d;
            (eu, el)
        } else {
            (f64::NAN, f64::NAN)
        }
    }

    fn candidate_o3s(&self, o4: &Point3) -> Option<(Point3, Point3)> {
        // Vector from the candidate point to the origin
        let v0 = -o4.coords;

        // Vector from the candidate point's projection on the XY plane to the origin
        let vp = Vector3::new(v0.x, v0.y, 0.0);

        // If the vector from the candidate point projection to the origin is zero
        // length, it means that the candidate point is directly above the robot origin,
        // and there's probably some sort of special case
        let e0 = v0.normalize();
        let e2 = v0.cross(&vp).normalize();
        let e1 = e2.cross(&e0).normalize();

        // let rot_m = Matrix3::from_columns(&[e0, e1, e2]);
        // let r = UnitQuaternion::from_matrix(&rot_m);
        // let t = Translation3::from(o4);
        // let tf = Frame3::from_parts(t, r).inverse();

        if let Some((a, h)) = self.corner_xy(v0.norm()) {
            let u = e0 * a + e1 * h;
            let l = e0 * a - e1 * h;
            Some((o4 + u, o4 + l))
        } else {
            None
        }
    }
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
