use crate::fanuc::crx::Crx;
use crate::{Frame3, Point3};

impl Crx {

    pub fn o4_circle(&self, end_frame: &Frame3) -> Vec<Point3> {
        // The radius is y1, the offset is x2
        let n = 500;
        let step = 2.0 * std::f64::consts::PI / n as f64;
        let mut result = Vec::with_capacity(n);
        let r = -self.p_vectors[4].y;
        let z = -self.p_vectors[5].x;
        println!("r: {}, z: {}", r, z);

        for i in 0..n {
            let angle = i as f64 * step;
            let x = r * angle.cos();
            let y = r * angle.sin();
            let point = Point3::new(x, y, z);
            result.push(end_frame * point);
        }

        result
    }

}