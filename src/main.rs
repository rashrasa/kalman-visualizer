extern crate nalgebra as na;

use std::{
    sync::{Arc, RwLock},
    thread,
    time::Instant,
};

use eframe::egui;
use egui::Color32;
use egui_plot::{Plot, Points};
use na::{Matrix2, Matrix2x1};

use kalman_visualizer::engine::{
    Integrator, Measure, Step, continuous::Continuous, sensor::SensorSpec,
};

fn main() -> eframe::Result {
    env_logger::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([640.0, 480.0]),
        ..Default::default()
    };
    let a = Matrix2::new(0.0, 1.0, -8.0 / 20.0, 0.0);
    let x0 = Matrix2x1::new(10.0, 0.0);
    let system_eul: Arc<RwLock<Continuous<2, 2>>> = Arc::new(RwLock::new(Continuous::new(
        Integrator::Euler,
        a.clone(),
        Matrix2x1::new(0.0, 0.0),
        Matrix2x1::new(0.0, 0.0),
        Matrix2x1::new(SensorSpec::new(10.0), SensorSpec::new(10.0)),
        Matrix2::new(1.0, 0.0, 0.0, 1.0),
        Matrix2x1::new(SensorSpec::new(0.0), SensorSpec::new(0.0)),
        x0.clone(),
    )));

    let system_rk4: Arc<RwLock<Continuous<2, 2>>> = Arc::new(RwLock::new(Continuous::new(
        Integrator::RK4,
        a.clone(),
        Matrix2x1::new(0.0, 0.0),
        Matrix2x1::new(0.0, 0.0),
        Matrix2x1::new(SensorSpec::new(10.0), SensorSpec::new(10.0)),
        Matrix2::new(1.0, 0.0, 0.0, 1.0),
        Matrix2x1::new(SensorSpec::new(0.0), SensorSpec::new(0.0)),
        x0.clone(),
    )));

    let system_eul_thread = system_eul.clone();
    let system_rk4_thread = system_rk4.clone();
    // No join since this runs infinitely along with the app
    thread::spawn(move || {
        let mut last = Instant::now();

        let mut sum_delta: f64 = 0.0;
        let h: f64 = 1.0 / 10.0;

        let mut last_update = Instant::now();

        let system_eul = system_eul_thread;
        let system_rk4 = system_rk4_thread;
        let mut system_eul_val = (*system_eul.read().unwrap()).clone();
        let mut system_rk4_val = (*system_rk4.read().unwrap()).clone();
        loop {
            sum_delta += last.elapsed().as_secs_f64();
            last = Instant::now();

            while sum_delta >= h {
                system_eul_val.step(h);
                system_rk4_val.step(h);
                sum_delta -= h;
            }
            if last_update.elapsed().as_secs_f64() > 1.0 / 240.0 {
                *system_eul.write().unwrap() = system_eul_val.clone();
                *system_rk4.write().unwrap() = system_rk4_val.clone();
                last_update = Instant::now();
            }
        }
    });
    let pos_points: Arc<RwLock<Vec<[f64; 2]>>> = Arc::new(RwLock::new(vec![]));
    let vel_points: Arc<RwLock<Vec<[f64; 2]>>> = Arc::new(RwLock::new(vec![]));

    let start = Instant::now();

    eframe::run_simple_native(
        "State Estimation Visualizer",
        options,
        move |ctx, _frame| {
            let system_eul_val = (*system_eul.read().unwrap()).clone();
            let system_rk4_val = (*system_rk4.read().unwrap()).clone();
            (*pos_points.write().unwrap()).push([
                system_eul_val.measure(0).unwrap(),
                system_eul_val.measure(1).unwrap(),
            ]);
            (*vel_points.write().unwrap()).push([
                system_rk4_val.measure(0).unwrap(),
                system_rk4_val.measure(1).unwrap(),
            ]);
            egui::CentralPanel::default().show(ctx, |ui| {
                ui.heading("State");
                ui.label(format!("Euler: {:.9}", system_eul_val.measure(0).unwrap()));
                ui.label(format!("RK4: {:.9}", system_rk4_val.measure(0).unwrap()));
                let plot = Plot::new("position_plot");
                plot.show(ui, |ui| {
                    for point in (*pos_points.read().unwrap()).iter() {
                        ui.points(
                            Points::new("position", *point).color(Color32::from_rgb(255, 0, 0)),
                        );
                    }
                    for point in (*vel_points.read().unwrap()).iter() {
                        ui.points(
                            Points::new("velocity", *point).color(Color32::from_rgb(0, 255, 0)),
                        );
                    }
                });
            });
            ctx.request_repaint();
        },
    )
}
