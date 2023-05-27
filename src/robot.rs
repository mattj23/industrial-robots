use k::{Chain, connect, JointType, NodeBuilder, Translation3, Vector3};

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
        .joint_type(JointType::Rotational { axis: Vector3::x_axis() })
        .into_node();

    let j5 = NodeBuilder::new()
        .name("j5")
        .translation(Translation3::new(335.0, 0.0, 0.0))
        .joint_type(JointType::Rotational { axis: Vector3::y_axis() })
        .into_node();

    let j6 = NodeBuilder::new()
        .name("j6")
        .translation(Translation3::new(80.0, 0.0, 0.0))
        .joint_type(JointType::Rotational { axis: Vector3::x_axis() })
        .into_node();

    connect![j1 => j2 => j3 => j4 => j5 => j6];

    Chain::from_root(j1)
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
}