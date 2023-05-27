use crate::poses::XyzWpr;
use k::{connect, Chain, JointType, NodeBuilder, Translation3, UnitQuaternion, Vector3, Isometry3};
use k::nalgebra::{Matrix4, try_convert};

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
        // self.chain.set_joint_positions_clamped(&rad_joints);
        self.chain.set_joint_positions_unchecked(&rad_joints);
    }

    pub fn poses(&self) -> Vec<XyzWpr> {
        let transforms = self.chain.update_transforms();
        transforms
            .iter()
            .map(|t| XyzWpr::from_isometry(t))
            .collect()
    }

    pub fn end_pose(&self) -> Isometry3<f64> {
        let transforms = self.chain.update_transforms();
        let last = transforms.last().unwrap();
        let m = Matrix4::new(0.0, 0.0, 1.0, 0.0,
                             0.0, -1.0, 0.0, 0.0,
                             1.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 1.0);
        // println!("last = {:?}", last.to_matrix());
        let working = last.to_matrix() * m;
        // println!("working = {:?}", working);
        let iso: Isometry3<f64> = try_convert(working).unwrap();
        // println!("iso = {:?}", iso);
        // XyzWpr::from_isometry(&iso)
        iso
    }
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

    let j5 = NodeBuilder::new()
        .name("j5")
        .translation(Translation3::new(335.0, 0.0, 0.0))
        .joint_type(JointType::Rotational {
            axis: -Vector3::y_axis(),
        })
        .into_node();

    let j6 = NodeBuilder::new()
        .name("j6")
        .translation(Translation3::new(80.0, 0.0, 0.0))
        .joint_type(JointType::Rotational {
            axis: -Vector3::x_axis(),
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
        let transform = XyzWpr::from_isometry(&robot.end_pose());
        let expected = XyzWpr::new(465.0, 0.0, 365.0, 180.0, -90.0, 0.0);
        assert!(transform.approx_eq(&expected, 1e-6));
    }

    #[test_case((-15.0, 0.0, 0.0, 0.0, 0.0, 0.0), (0.0, -0.2588190451, 0.9659258263, 449.1555092244, -0.0, -0.9659258263, -0.2588190451, -120.3508559727, 1.0, -0.0, -0.0, 365.0, 0.0, 0.0, 0.0, 1.0))]
    #[test_case((20.0, 0.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.3420201433, 0.9396926208, 436.9570686654, -0.0, -0.9396926208, 0.3420201433, 159.0393666464, 1.0, -0.0, -0.0, 365.0, 0.0, 0.0, 0.0, 1.0))]
    #[test_case((0.0, -15.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 1.0, 379.5897151162, -0.0, -1.0, 0.0, 0.0, 1.0, -0.0, -0.0, 353.7555226754, 0.0, 0.0, 0.0, 1.0))]
    #[test_case((0.0, 20.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 1.0, 577.8666472975, -0.0, -1.0, 0.0, 0.0, 1.0, -0.0, -0.0, 345.0985648593, 0.0, 0.0, 0.0, 1.0))]
    #[test_case((0.0, 0.0, -15.0, 0.0, 0.0, 0.0), (0.2588190451, 0.0, 0.9659258263, 459.9178844886, -0.0, -1.0, 0.0, 0.0, 0.9659258263, -0.0, -0.2588190451, 256.3975002026, 0.0, 0.0, 0.0, 1.0))]
    #[test_case((0.0, 0.0, 20.0, 0.0, 0.0, 0.0), (-0.3420201433, 0.0, 0.9396926208, 428.0017326098, -0.0, -1.0, 0.0, -0.0, 0.9396926208, -0.0, 0.3420201433, 504.8276012077, 0.0, 0.0, 0.0, 1.0))]
    #[test_case((0.0, 0.0, 0.0, -15.0, 0.0, 0.0), (0.0, 0.0, 1.0, 465.0, -0.2588190451, -0.9659258263, 0.0, 0.0, 0.9659258263, -0.2588190451, -0.0, 365.0, 0.0, 0.0, 0.0, 1.0))]
    #[test_case((0.0, 0.0, 0.0, 20.0, 0.0, 0.0), (0.0, 0.0, 1.0, 465.0, 0.3420201433, -0.9396926208, 0.0, 0.0, 0.9396926208, 0.3420201433, -0.0, 365.0, 0.0, 0.0, 0.0, 1.0))]
    #[test_case((0.0, 0.0, 0.0, 0.0, -15.0, 0.0), (0.2588190451, 0.0, 0.9659258263, 462.2740661031, -0.0, -1.0, 0.0, 0.0, 0.9659258263, -0.0, -0.2588190451, 344.2944763918, 0.0, 0.0, 0.0, 1.0))]
    #[test_case((0.0, 0.0, 0.0, 0.0, 20.0, 0.0), (-0.3420201433, 0.0, 0.9396926208, 460.1754096629, -0.0, -1.0, 0.0, 0.0, 0.9396926208, -0.0, 0.3420201433, 392.3616114661, 0.0, 0.0, 0.0, 1.0))]
    #[test_case((0.0, 0.0, 0.0, 0.0, 0.0, -15.0), (0.0, 0.0, 1.0, 465.0, -0.2588190451, -0.9659258263, 0.0, 0.0, 0.9659258263, -0.2588190451, -0.0, 365.0, 0.0, 0.0, 0.0, 1.0))]
    #[test_case((0.0, 0.0, 0.0, 0.0, 0.0, 20.0), (0.0, 0.0, 1.0, 465.0, 0.3420201433, -0.9396926208, 0.0, 0.0, 0.9396926208, 0.3420201433, -0.0, 365.0, 0.0, 0.0, 0.0, 1.0))]
    fn fanuc_single_joint(jf: (f64, f64, f64, f64, f64, f64), mf: (f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64)) {
        let mut robot = FanucLrMate200id::new();
        robot.set_joints(&[jf.0, jf.1, jf.2, jf.3, jf.4, jf.5]);
        let transform = robot.end_pose();
        let expected = Matrix4::new(mf.0, mf.1, mf.2, mf.3, mf.4, mf.5, mf.6, mf.7, mf.8, mf.9, mf.10, mf.11, mf.12, mf.13, mf.14, mf.15);
        let check = transform.to_matrix();

        assert_relative_eq!(check, expected, epsilon = 1e-6);
    }
}
