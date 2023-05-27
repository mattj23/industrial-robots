use k::*;
use k::nalgebra::{Matrix4, Vector3, Translation3, try_convert};
use k::nalgebra::geometry::{IsometryMatrix3, Isometry3};


fn main() {
    let joints = &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let m4 = Matrix4::new(0.0, 0.0, 1.0, 465.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 365.0, 0.0, 0.0, 0.0, 1.0);
    println!("{:?}", m4);
    let expected: Isometry3<f64> = try_convert(m4).unwrap();

    let m4_ = Matrix4::new(0.9572590453, -0.2498925755, 0.1456324858, 59.5794643289, -0.2886479285, -0.7934076632, 0.5358979878, 6.7548483372, -0.0183709982, -0.5550297115, -0.8316276365, -88.8341095469, 0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000);
    println!("{:?}", m4_);
    let expected_: Isometry3<f64> = try_convert(m4_).unwrap();

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

    let mut tree = Chain::from_root(j1);
    let transforms = tree.update_transforms();
    for t in transforms {
        println!("before {:?}", t);
    }

    println!("-------------------");

    // tree.set_joint_positions_clamped(&[f64::pi() / 2.0, 0.0]);
    tree.set_joint_positions_clamped(&[rad(0.0), rad(0.0), rad(0.0), rad(0.0), rad(0.0), rad(0.0)]);
    let transforms = tree.update_transforms();
    for t in transforms {
        println!("after {:?}", t);
    }

    println!("expected: {:?}", expected);
}

fn rad(deg: f64) -> f64 {
    deg * f64::pi() / 180.0
}