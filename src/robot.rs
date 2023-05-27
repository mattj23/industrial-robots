use k::{Chain, connect, JointType, NodeBuilder, Translation3, Vector3};
use crate::poses::XyzWpr;

pub fn fanuc_lr_mate_200id() -> Chain<f64> {
    let j1 = NodeBuilder::new()
        .name("j1")
        .joint_type(JointType::Rotational { axis: Vector3::z_axis() })
        .into_node();

    let j2 = NodeBuilder::new()
        .name("j2")
        .translation(Translation3::new(50.0, 0.0, 0.0))
        .joint_type(JointType::Rotational { axis: Vector3::y_axis() })
        .into_node();

    let j3 = NodeBuilder::new()
        .name("j3")
        .translation(Translation3::new(0.0, 0.0, 330.0))
        .joint_type(JointType::Rotational { axis: -Vector3::y_axis() })
        .into_node();

    let j4 = NodeBuilder::new()
        .name("j4")
        .translation(Translation3::new(0.0, 0.0, 35.0))
        .joint_type(JointType::Rotational { axis: -Vector3::x_axis() })
        .into_node();

    let j5 = NodeBuilder::new()
        .name("j5")
        .translation(Translation3::new(335.0, 0.0, 0.0))
        .joint_type(JointType::Rotational { axis: -Vector3::y_axis() })
        .into_node();

    let j6 = NodeBuilder::new()
        .name("j6")
        .translation(Translation3::new(80.0, 0.0, 0.0))
        .joint_type(JointType::Rotational { axis: Vector3::x_axis() })
        .into_node();

    connect![j1 => j2 => j3 => j4 => j5 => j6];

    Chain::from_root(j1)
}

fn fanuc_joints_to_rad(joints: &[f64]) -> [f64; 6] {
    let mut rad_joints = [0.0; 6];
    for (i, j) in joints.iter().enumerate() {
        rad_joints[i] = j.to_radians();
    }
    rad_joints[2] += rad_joints[1];
    rad_joints
}

#[cfg(test)]
mod tests {
    use k::Isometry3;
    use k::nalgebra::{Matrix4, try_convert};
    use super::*;
    use approx::assert_relative_eq;
    use test_case::test_case;

    #[test]
    fn fanuc_at_home() {
        let joints = &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let m4 = Matrix4::new(0.0, 0.0, 1.0, 465.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 365.0, 0.0, 0.0, 0.0, 1.0);
        let expected: Isometry3<f64> = try_convert(m4).unwrap();

        let mut chain = fanuc_lr_mate_200id();
        let transform = chain.update_transforms().last().unwrap().clone();
        assert_relative_eq!(transform.translation.x, expected.translation.x, epsilon = 1e-6);
        assert_relative_eq!(transform.translation.y, expected.translation.y, epsilon = 1e-6);
        assert_relative_eq!(transform.translation.z, expected.translation.z, epsilon = 1e-6);
    }

    #[test_case((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (465.0, 0.0, 365.0, -180.0, -90.0, 0.0))]
    #[test_case((-15.0, 0.0, 0.0, 0.0, 0.0, 0.0), (449.1555092244, -120.3508559727, 365.0, 165.0, -90.0, 0.0))]
    #[test_case((20.0, 0.0, 0.0, 0.0, 0.0, 0.0), (436.9570686654, 159.0393666464, 365.0, -160.0, -90.0, 0.0))]
    #[test_case((0.0, -15.0, 0.0, 0.0, 0.0, 0.0), (379.5897151162, 0.0, 353.7555226754, -180.0, -90.0, 0.0))]
    #[test_case((0.0, 20.0, 0.0, 0.0, 0.0, 0.0), (577.8666472975, 0.0, 345.0985648593, -180.0, -90.0, 0.0))]
    #[test_case((0.0, 0.0, -15.0, 0.0, 0.0, 0.0), (459.9178844886, 0.0, 256.3975002026, -0.0, -75.0, -180.0))]
    #[test_case((0.0, 0.0, 20.0, 0.0, 0.0, 0.0), (428.0017326098, -0.0, 504.8276012077, -180.0, -70.0, -0.0))]
    #[test_case((0.0, 0.0, 0.0, -15.0, 0.0, 0.0), (465.0, 0.0, 365.0, -90.0, -75.0, -90.0))]
    #[test_case((0.0, 0.0, 0.0, 20.0, 0.0, 0.0), (465.0, 0.0, 365.0, 90.0, -70.0, 90.0))]
    #[test_case((0.0, 0.0, 0.0, 0.0, -15.0, 0.0), (462.2740661031, 0.0, 344.2944763918, -0.0, -75.0, -180.0))]
    #[test_case((0.0, 0.0, 0.0, 0.0, 20.0, 0.0), (460.1754096629, 0.0, 392.3616114661, -180.0, -70.0, -0.0))]
    #[test_case((0.0, 0.0, 0.0, 0.0, 0.0, -15.0), (465.0, 0.0, 365.0, -90.0, -75.0, -90.0))]
    #[test_case((0.0, 0.0, 0.0, 0.0, 0.0, 20.0), (465.0, 0.0, 365.0, 90.0, -70.0, 90.0))]
    fn fanuc_single_joint_directions(jf: (f64, f64, f64, f64, f64, f64), pf: (f64, f64, f64, f64, f64, f64)) {
        let mut chain = fanuc_lr_mate_200id();
        let joints = fanuc_joints_to_rad(&[jf.0, jf.1, jf.2, jf.3, jf.4, jf.5]);
        chain.set_joint_positions_clamped(&joints);
        let transform = XyzWpr::from_isometry(chain.update_transforms().last().unwrap());
        assert_relative_eq!(transform.x, pf.0, epsilon = 1e-6);
        assert_relative_eq!(transform.y, pf.1, epsilon = 1e-6);
        assert_relative_eq!(transform.z, pf.2, epsilon = 1e-6);
        // assert_relative_eq!(transform.rotation.euler_angles().0, pf.3, epsilon = 1e-6);
        // assert_relative_eq!(transform.rotation.euler_angles().1, pf.4, epsilon = 1e-6);
        // assert_relative_eq!(transform.rotation.euler_angles().2, pf.5, epsilon = 1e-6);

    }
}