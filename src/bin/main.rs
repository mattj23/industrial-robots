// use industrial_robots::robot::fanuc_lr_mate_200id2;
use industrial_robots::poses::XyzWpr;

use k::nalgebra::geometry::{Isometry3, IsometryMatrix3};
use k::nalgebra::{try_convert, Matrix4, Translation3, Vector3};
use k::*;

fn main() {
    // let options = vec![180, 90, -90, 0];
    // for a in options.iter() {
    //     for b in options.iter() {
    //         for c in options.iter() {
    //             let chain = fanuc_lr_mate_200id2(rad(*a as f64), rad(*b as f64), rad(*c as f64));
    //             let transform = chain.update_transforms().last().unwrap().clone();
    //             let pose = XyzWpr::from_isometry(&transform);
    //             if pose.r.abs() > 0.0001 {
    //                 continue;
    //             }
    //             println!("pose: w={}, p={}, r={}", pose.w, pose.p, pose.r);
    //         }
    //     }
    // }
    // let mut chain = fanuc_lr_mate_200id2(0.0, rad(-90.0), rad(180.0));
    // chain.set_joint_positions_clamped(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    // let transform = chain.update_transforms().last().unwrap().clone();
    // let pose = XyzWpr::from_isometry(&transform);
    // println!("pose: w={}, p={}, r={}", pose.w, pose.p, pose.r);

    let p0 = XyzWpr::new(0.0, 0.0, 0.0, 180.0, -90.0, 0.0);
    let p1 = XyzWpr::new(0.0, 0.0, 0.0, 0.0, -90.0, 180.0);
    println!("p0: {:?}", p0.to_isometry());
    println!("p1: {:?}", p1.to_isometry());
    println!("approx: {}", p0.approx_eq(&p1, 0.0001));




}

fn rad(deg: f64) -> f64 {
    deg * f64::pi() / 180.0
}
