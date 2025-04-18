pub mod fanuc;
mod frames;
mod helpers;
mod type_aliases;
pub mod micro_mesh;
mod collision;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

// Re-export nalgebra and parry to help consuming crates manage dependencies
pub use parry3d_f64;
pub use parry3d_f64::na as nalgebra;

// Re-export type aliases and pose types
pub use frames::XyzWpr;
pub use type_aliases::*;
pub use collision::{CollisionScene, TriMesh};
