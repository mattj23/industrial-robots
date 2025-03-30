pub mod fanuc;
mod frames;
mod helpers;
pub mod robot;
mod type_aliases;
pub mod micro_mesh;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

// Re-export nalgebra and ik_geo to help consuming crates manage dependencies
pub use ik_geo;
pub use ik_geo::nalgebra;

// Re-export type aliases and pose types
pub use frames::XyzWpr;
pub use type_aliases::*;
