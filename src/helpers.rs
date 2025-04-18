use crate::Result;
use crate::type_aliases::{Frame3, Vector3};
use crate::nalgebra::{Matrix3, Translation3, UnitQuaternion, try_convert, Matrix4};

pub fn parts_to_iso(rot: Matrix3<f64>, trans: Vector3) -> Frame3 {
    let r = UnitQuaternion::from_matrix(&rot);
    let t = Translation3::from(trans);

    Frame3::from_parts(t, r)
}

pub fn iso_to_parts(iso: &Frame3) -> (Matrix3<f64>, Vector3) {
    let t = iso.translation.vector;
    let r = iso.rotation.to_rotation_matrix();

    (r.into(), t)
}

pub fn row_slice_to_iso(slice: &[f64]) -> Result<Frame3> {
    if slice.len() != 16 {
        return Err("Slice length must be 16".into());
    }

    let m = Matrix4::new(
        slice[0], slice[1], slice[2], slice[3], slice[4], slice[5], slice[6], slice[7], slice[8],
        slice[9], slice[10], slice[11], slice[12], slice[13], slice[14], slice[15],
    );

    try_convert(m).ok_or("Failed to convert matrix to isometry".into())
}
