use std::{
    f64::consts::PI,
    io::Read,
    sync::{
        Arc, RwLock,
        mpsc::{Sender, channel},
    },
    thread::{self, JoinHandle},
};

use log::info;
use na::{ArrayStorage, Const, Matrix};

use crate::{engine::create_event_loop, environment::input::InputHandler};

pub struct Car {
    // constants
    max_acceleration_abs: f64,
    max_braking_abs: f64,
    max_speed: f64,

    // state
    // x, y, theta
    position: Matrix<f64, Const<3>, Const<1>, ArrayStorage<f64, 3, 1>>,
    velocity: Matrix<f64, Const<3>, Const<1>, ArrayStorage<f64, 3, 1>>,
    acceleration: Matrix<f64, Const<3>, Const<1>, ArrayStorage<f64, 3, 1>>,

    cruise_control: bool,
    cruise_control_set_point: f64,
}

impl Car {
    pub fn accelerate(&mut self, dt: f64) {
        todo!();
    }

    pub fn cruise_control_enable(&mut self) {
        self.cruise_control_set_point =
            (self.velocity[0].powi(2) * self.velocity[1].powi(2)).sqrt();
        self.cruise_control = true;
    }

    pub fn cruise_control_set_point(&mut self, set_point: f64) {
        self.cruise_control_set_point = set_point;
    }

    pub fn cruise_control_disable(&mut self) {
        self.cruise_control = false;
    }
}

pub struct CarHandler {
    input_sender: Sender<char>,
}

impl Car {
    pub fn spawn(
        initial_position: (f64, f64),
        max_acceleration_abs: f64,
        max_braking_abs: f64,
        max_speed: f64,
        polling_rate: u32,
        input_handler: &InputHandler,
    ) -> CarHandler {
        let car = Car {
            position: Matrix::<f64, Const<3>, Const<1>, ArrayStorage<f64, 3, 1>>::new(
                initial_position.0,
                initial_position.1,
                PI / 2.0,
            ),
            velocity: Matrix::<f64, Const<3>, Const<1>, ArrayStorage<f64, 3, 1>>::new(
                0.0, 0.0, 0.0,
            ),
            acceleration: Matrix::<f64, Const<3>, Const<1>, ArrayStorage<f64, 3, 1>>::new(
                0.0, 0.0, 0.0,
            ),
            max_acceleration_abs: max_acceleration_abs.abs(),
            max_braking_abs: max_braking_abs.abs(),
            max_speed: max_speed,

            cruise_control: false,
            cruise_control_set_point: 0.0,
        };

        let (tx, rx) = channel::<char>();

        let car_handle = thread::spawn(|| {
            let car = Arc::new(RwLock::new(car));

            let car1 = car.clone();

            let car_input_handler = thread::spawn(move || {
                let car = car1;
                let mut buf: [u8; 1024] = [0; 1024];
                loop {
                    let len = std::io::stdin().lock().read(&mut buf).unwrap();
                    info!("{}", std::str::from_utf8(&buf[0..len]).unwrap())
                }
            });
            create_event_loop(60.0, Box::new(move |dt| {}))();
        });

        CarHandler { input_sender: tx }
    }
}

impl CarHandler {}
