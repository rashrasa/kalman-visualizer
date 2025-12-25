#[derive(Clone, Debug, PartialEq)]
pub struct SensorSpec {
    variance: f64,
}

impl SensorSpec {
    pub const fn new(variance: f64) -> Self {
        SensorSpec { variance }
    }
    pub fn variance(&self) -> f64 {
        return self.variance;
    }
}
