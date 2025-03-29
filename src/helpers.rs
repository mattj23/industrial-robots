use crate::type_aliases::{Iso3, Vector3};
use crate::Result;
use ik_geo::nalgebra::{try_convert, Matrix3, Translation3, UnitQuaternion};
use crate::nalgebra::Matrix4;

pub fn parts_to_iso(rot: Matrix3<f64>, trans: Vector3) -> Iso3 {
    let r = UnitQuaternion::from_matrix(&rot);
    let t = Translation3::from(trans);

    Iso3::from_parts(t, r)
}

pub fn row_slice_to_iso(slice: &[f64]) -> Result<Iso3> {
    if slice.len() != 16 {
        return Err("Slice length must be 16".into());
    }

    let m = Matrix4::new(
        slice[0], slice[1], slice[2], slice[3],
        slice[4], slice[5], slice[6], slice[7],
        slice[8], slice[9], slice[10], slice[11],
        slice[12], slice[13], slice[14], slice[15],
    );

    try_convert(m).ok_or("Failed to convert matrix to isometry".into())
}