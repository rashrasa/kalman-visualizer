extern crate nalgebra as na;

use std::{
    sync::{Arc, RwLock},
    thread,
    time::Instant,
};

use eframe::egui;
use egui_plot::{Plot, Points};
use na::{Matrix1x2, Matrix2, Matrix2x1};

use kalman_visualizer::core::Step;
use kalman_visualizer::systems::msd;

fn main() -> eframe::Result {
    env_logger::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([640.0, 480.0]),
        ..Default::default()
    };
    // Mass-Spring-Damper
    const M: f64 = 10.0;
    const K: f64 = 10.0;

    let msd = Arc::new(RwLock::new(msd::MSD::underdamped(
        M,
        K,
        Matrix2x1::new(0.0, 12.0),
    )));

    let msd = Arc::new(RwLock::new(msd::MSD::manual(
        Matrix2::new(0.0, 1.0, 0.0, 0.0),
        Matrix2x1::new(0.0, 0.0),
        Matrix1x2::new(0.0, 0.0),
        Matrix2x1::new(0.0, 0.0),
        Matrix2x1::new(0.0, 1.0),
    )));

    let msd_thread = msd.clone();

    // No join since this runs infinitely along with the app
    thread::spawn(move || {
        let mut last = Instant::now();

        let mut sum_delta: f64 = 0.0;
        let h: f64 = 1.0 / 10000.0;

        let mut last_update = Instant::now();

        let msd = msd_thread;
        let mut msd_val = (*msd.read().unwrap()).clone();
        loop {
            sum_delta += last.elapsed().as_secs_f64();
            last = Instant::now();

            while sum_delta >= h {
                msd_val.step(h);
                sum_delta -= h;
            }
            if last_update.elapsed().as_secs_f64() > 1.0 / 240.0 {
                *msd.write().unwrap() = msd_val.clone();
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
            let msd_val = (*msd.read().unwrap()).clone();
            (*pos_points.write().unwrap()).push([start.elapsed().as_secs_f64(), msd_val.x.x]);
            (*vel_points.write().unwrap()).push([start.elapsed().as_secs_f64(), msd_val.x.y]);
            egui::CentralPanel::default().show(ctx, |ui| {
                ui.heading("State");
                ui.label(format!("Position: {:.9}", msd_val.x.x));
                ui.label(format!("Velocity: {:.9}", msd_val.x.y));
                let plot = Plot::new("position_plot");
                plot.show(ui, |ui| {
                    for point in (*pos_points.read().unwrap()).iter() {
                        ui.points(Points::new("position", *point));
                    }
                    for point in (*vel_points.read().unwrap()).iter() {
                        ui.points(Points::new("velocity", *point));
                    }
                });
            });
            ctx.request_repaint();
        },
    )
}
