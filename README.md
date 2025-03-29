# Industrial Robots

The goal of this crate is to provide known correct kinematic models for common 6-axis industrial robot arms based on the [IK-Geo](https://github.com/rpiRobotics/ik-geo) unified inverse kinematics library in order to allow others to quickly set up simulations, studies, and applications with these robots without having to dig through spec sheets and cross-reference library documentation to get underway.

Right now, I'm implementing a few of the common FANUC arms because that's what I use and have access to. However, I welcome any contributions for other arms or robot types which are known to work.

## Supported Arms

This library is very early in progress.

### FANUC LR Mate 200iD

(In progress)

### FANUC CRX Series

The FANUC CRX series is a family of collaborative 6-axis arms with identical joint structures ranging from 5kg to 30kg (as of Q1 2025) payloads.  They have non-spherical wrists and three parallel joint axes. 

Currently, the 5iA and 10iA have kinematics checked. 

```rust 
use industrial_robots::fanuc::Crx;

fn main() {
    let robot = Crx::new_5ia();
    
}
```