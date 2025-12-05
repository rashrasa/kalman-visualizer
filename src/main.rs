extern crate nalgebra as na;
use na::{Matrix2, Vector2};

const H: f32 = 0.0001;
const ITERATIONS: i32 = (1.0 / H) as i32;

fn main() {
    let mut x = Vector2::new(0.0, 0.0);
    let a = Matrix2::new(0.0, 1.0, 0.0, 0.0);
    let k = Vector2::new(0.0, -9.81);

    for i in 0..10 * ITERATIONS {
        if i % ITERATIONS == 0 {
            print!("{}\n", x.x);
        }
        // euler
        x = x + H * (a * x + k);
    }
}
