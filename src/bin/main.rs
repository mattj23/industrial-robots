use industrial_robots::poses::XyzWpr;
use industrial_robots::robot::FanucLrMate200id;

use k::nalgebra::geometry::{Isometry3, IsometryMatrix3};
use k::nalgebra::{try_convert, Matrix4, Translation3, Vector3};
use k::*;

fn main() {
    let mut robot = FanucLrMate200id::new();
    robot.set_joints(&[80.97, 27.58, -47.58, 0.27, -46.98, 89.27]);
    let pose = robot.end_pose();
    println!("pose: {:?}", pose);

    // let target = Isometry3::translation(0.0, -10.0, -5.0) * pose;
    let target = pose.clone();
    println!("target: {:?}", pose);
    if let Some(joints) = robot.find_joints(&target) {
        println!("joints: {:?}", joints);
    } else {
        println!("No solution found");
    }
}

fn rad(deg: f64) -> f64 {
    deg * f64::pi() / 180.0
}
