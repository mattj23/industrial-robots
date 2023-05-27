use crate::poses::XyzWpr;
use k::{connect, Chain, JointType, NodeBuilder, Translation3, UnitQuaternion, Vector3};

pub struct FanucLrMate200id {
    chain: Chain<f64>,
}

impl FanucLrMate200id {
    pub fn new() -> Self {
        let chain = fanuc_lr_mate_200id();
        Self { chain }
    }

    pub fn set_joints(&mut self, joints: &[f64]) {
        let rad_joints = fanuc_joints_to_rad(joints);
        self.chain.set_joint_positions_clamped(&rad_joints);
    }

    pub fn poses(&self) -> Vec<XyzWpr> {
        let transforms = self.chain.update_transforms();
        transforms
            .iter()
            .map(|t| XyzWpr::from_isometry(t))
            .collect()
    }

    pub fn end_pose(&self) -> XyzWpr {
        let transforms = self.chain.update_transforms();
        let last = transforms.last().unwrap();
        XyzWpr::from_isometry(last)
    }
}

pub fn fanuc_lr_mate_200id2(a: f64, b: f64, c: f64) -> Chain<f64> {
    let j1 = NodeBuilder::new()
        .name("j1")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .into_node();

    let j2 = NodeBuilder::new()
        .name("j2")
        .translation(Translation3::new(50.0, 0.0, 0.0))
        .joint_type(JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .into_node();

    let j3 = NodeBuilder::new()
        .name("j3")
        .translation(Translation3::new(0.0, 0.0, 330.0))
        .joint_type(JointType::Rotational {
            axis: -Vector3::y_axis(),
        })
        .into_node();

    let j4 = NodeBuilder::new()
        .name("j4")
        .translation(Translation3::new(0.0, 0.0, 35.0))
        .joint_type(JointType::Rotational {
            axis: -Vector3::x_axis(),
        })
        .into_node();

    let basis = UnitQuaternion::from_euler_angles(a, b, c);

    let j5 = NodeBuilder::new()
        .name("j5")
        .translation(Translation3::new(335.0, 0.0, 0.0))
        .rotation(basis)
        .joint_type(JointType::Rotational {
            axis: -Vector3::y_axis(),
        })
        .into_node();

    let j6 = NodeBuilder::new()
        .name("j6")
        .translation(Translation3::new(80.0, 0.0, 0.0))
        .joint_type(JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .into_node();

    connect![j1 => j2 => j3 => j4 => j5 => j6];

    Chain::from_root(j1)
}

fn fanuc_lr_mate_200id() -> Chain<f64> {
    let j1 = NodeBuilder::new()
        .name("j1")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .into_node();

    let j2 = NodeBuilder::new()
        .name("j2")
        .translation(Translation3::new(50.0, 0.0, 0.0))
        .joint_type(JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .into_node();

    let j3 = NodeBuilder::new()
        .name("j3")
        .translation(Translation3::new(0.0, 0.0, 330.0))
        .joint_type(JointType::Rotational {
            axis: -Vector3::y_axis(),
        })
        .into_node();

    let j4 = NodeBuilder::new()
        .name("j4")
        .translation(Translation3::new(0.0, 0.0, 35.0))
        .joint_type(JointType::Rotational {
            axis: -Vector3::x_axis(),
        })
        .into_node();

    let p = XyzWpr::new(0.0, 0.0, 0.0, 180.0, -90.0, 0.0);
    let basis = p.to_isometry().rotation;

    let j5 = NodeBuilder::new()
        .name("j5")
        .translation(Translation3::new(335.0, 0.0, 0.0))
        .rotation(basis)
        .joint_type(JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .into_node();

    let j6 = NodeBuilder::new()
        .name("j6")
        .translation(Translation3::new(0.0, 0.0, 80.0))
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
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
    use super::*;
    use approx::assert_relative_eq;
    use k::nalgebra::{try_convert, Matrix4};
    use k::Isometry3;
    use test_case::test_case;

    #[test]
    fn fanuc_at_home() {
        let mut robot = FanucLrMate200id::new();
        robot.set_joints(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let transform = robot.end_pose();
        let expected = XyzWpr::new(465.0, 0.0, 365.0, 180.0, -90.0, 0.0);
        assert!(transform.approx_eq(&expected, 1e-6));
    }

    // #[test_case((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (465.0, 0.0, 365.0, -180.0, -90.0, 0.0))]
    #[test_case((-15.0, 0.0, 0.0, 0.0, 0.0, 0.0), (449.1555092244, -120.3508559727, 365.0, 165.0, -90.0, 0.0))] //what
    // #[test_case((20.0, 0.0, 0.0, 0.0, 0.0, 0.0), (436.9570686654, 159.0393666464, 365.0, -160.0, -90.0, 0.0))] //what
    // #[test_case((0.0, -15.0, 0.0, 0.0, 0.0, 0.0), (379.5897151162, 0.0, 353.7555226754, -180.0, -90.0, 0.0))]
    // #[test_case((0.0, 20.0, 0.0, 0.0, 0.0, 0.0), (577.8666472975, 0.0, 345.0985648593, -180.0, -90.0, 0.0))]
    // #[test_case((0.0, 0.0, -15.0, 0.0, 0.0, 0.0), (459.9178844886, 0.0, 256.3975002026, -0.0, -75.0, -180.0))]
    // #[test_case((0.0, 0.0, 20.0, 0.0, 0.0, 0.0), (428.0017326098, -0.0, 504.8276012077, -180.0, -70.0, -0.0))]
    // #[test_case((1.0, 0.0, 0.0, -15.0, 0.0, 0.0), (465.0, 0.0, 365.0, -90.0, -75.0, -90.0))] // what
    // #[test_case((0.0, 0.0, 0.0, 20.0, 0.0, 0.0), (465.0, 0.0, 365.0, 90.0, -70.0, 90.0))] // what
    // #[test_case((0.0, 0.0, 0.0, 0.0, -15.0, 0.0), (462.2740661031, 0.0, 344.2944763918, -0.0, -75.0, -180.0))]
    // #[test_case((0.0, 0.0, 0.0, 0.0, 20.0, 0.0), (460.1754096629, 0.0, 392.3616114661, -180.0, -70.0, -0.0))]
    // #[test_case((0.0, 0.0, 0.0, 0.0, 0.0, -15.0), (465.0, 0.0, 365.0, -90.0, -75.0, -90.0))]
    // #[test_case((0.0, 0.0, 0.0, 0.0, 0.0, 20.0), (465.0, 0.0, 365.0, 90.0, -70.0, 90.0))]
    fn fanuc_single_joint_directions(
        jf: (f64, f64, f64, f64, f64, f64),
        pf: (f64, f64, f64, f64, f64, f64),
    ) {
        let mut robot = FanucLrMate200id::new();
        robot.set_joints(&[jf.0, jf.1, jf.2, jf.3, jf.4, jf.5]);
        let transform = robot.end_pose();
        let expected = XyzWpr::new(pf.0, pf.1, pf.2, pf.3, pf.4, pf.5);
        assert!(transform.approx_eq(&expected, 1e-6));
        // assert_relative_eq!(transform.x, pf.0, epsilon = 1e-6);
        // assert_relative_eq!(transform.y, pf.1, epsilon = 1e-6);
        // assert_relative_eq!(transform.z, pf.2, epsilon = 1e-6);
        // assert_relative_eq!(transform.rotation.euler_angles().0, pf.3, epsilon = 1e-6);
        // assert_relative_eq!(transform.rotation.euler_angles().1, pf.4, epsilon = 1e-6);
        // assert_relative_eq!(transform.rotation.euler_angles().2, pf.5, epsilon = 1e-6);
    }
}
