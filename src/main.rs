extern crate nalgebra as na;

use std::{
    sync::{Arc, RwLock},
    thread,
    time::{Duration, Instant},
};

use eframe::egui;
use egui_plot::{Plot, Points};
use na::{Matrix2, Vector2};

fn main() -> eframe::Result {
    env_logger::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([640.0, 480.0]),
        ..Default::default()
    };
    let x = Arc::new(RwLock::new(Vector2::new(0.0, 0.0)));
    let a = Matrix2::new(0.0, 1.0, 0.0, 0.0);
    let k = Vector2::new(0.0, -9.81);

    let x_thread = x.clone();

    // No join since this runs infinitely along with the app
    thread::spawn(move || {
        let mut last = Instant::now();

        let mut sum_delta: f64 = 0.0;
        let h: f64 = 1.0 / 200.0;

        let mut last_update = Instant::now();

        let x = x_thread;
        let mut x_val = (*x.read().unwrap()).clone();
        loop {
            sum_delta += last.elapsed().as_secs_f64();
            last = Instant::now();

            while sum_delta >= h {
                // euler
                x_val = x_val + h * (a * x_val + k);
                sum_delta -= h;
            }
            if last_update.elapsed().as_secs_f64() > 1.0 / 20.0 {
                *x.write().unwrap() = x_val;
                last_update = Instant::now();
            }
            thread::sleep(Duration::from_millis(0));
        }
    });
    let pos_points: Arc<RwLock<Vec<[f64; 2]>>> = Arc::new(RwLock::new(vec![]));
    let vel_points: Arc<RwLock<Vec<[f64; 2]>>> = Arc::new(RwLock::new(vec![]));

    let start = Instant::now();
    eframe::run_simple_native(
        "State Estimation Visualizer",
        options,
        move |ctx, _frame| {
            let x_val = *x.read().unwrap();
            (*pos_points.write().unwrap()).push([start.elapsed().as_secs_f64(), x_val.x]);
            (*vel_points.write().unwrap()).push([start.elapsed().as_secs_f64(), x_val.y]);
            egui::CentralPanel::default().show(ctx, |ui| {
                ui.heading("State");
                ui.label(format!("Position: {:.1}", x_val.x));
                ui.label(format!("Velocity: {:.1}", x_val.y));
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
