use industrial_robots::Frame3;
use industrial_robots::nalgebra::{Matrix4, try_convert};

fn main() {
    let c: Frame3 = try_convert(Matrix4::new(
        0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ))
    .unwrap();

    let r = c.rotation.axis_angle().unwrap();

    println!("{:?}", c);
    println!("{:?}", r);
    println!("{:?}", r.0.into_inner() * r.1);
}
