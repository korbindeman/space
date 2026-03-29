use glam::DVec3;

#[derive(Clone, Debug)]
pub struct SimState {
    pub positions: Vec<DVec3>,
    pub velocities: Vec<DVec3>,
    pub masses: Vec<f64>,
}

impl SimState {
    pub fn body_count(&self) -> usize {
        self.positions.len()
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct NodeId(pub u64);
