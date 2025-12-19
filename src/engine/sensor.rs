#[derive(Clone, Debug)]
pub struct SensorSpec {
    variance: f64,
}

impl SensorSpec {
    pub fn new(variance: f64) -> Self {
        SensorSpec{variance}
    }
    pub fn sigma_y(&self) -> f64 {
        return self.variance;
    }
}