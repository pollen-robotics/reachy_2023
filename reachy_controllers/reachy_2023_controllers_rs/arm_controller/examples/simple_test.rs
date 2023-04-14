use ::arm_controller::ArmController;
use arm_controller::PID;
use std::{
    error::Error,
    f64::consts::PI,
    thread,
    time::{Duration, SystemTime},
};

fn main() -> Result<(), Box<dyn Error>> {
    let mut arm_controller = ArmController::new(
        "/dev/ttyACM0",
        [10, 11, 12, 13, 14, 15, 16, 17],
        [PI / 2.0, PI / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [false, false, false, false, false, false, false, true],
        [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -2.0],
        40,
        41,
    )?;

    let mut i = 1;
    let mut read_error = 0;
    // let mut write_error = 0;

    loop {
        thread::sleep(Duration::from_millis(1000));
        if let Ok(res) = arm_controller.get_mx_moving_speed() {
            println!("READ {:?}", res);
        }
        if let Ok(res) = arm_controller.get_mx_torque_limit() {
            println!("READ {:?}", res);
        }
    }
}
