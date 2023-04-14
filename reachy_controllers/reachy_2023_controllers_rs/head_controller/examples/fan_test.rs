use ::head_controller::HeadController;
use head_controller::PID;
use std::{
    error::Error,
    f64::consts::PI,
    thread,
    time::{Duration, SystemTime},
};

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();

    let mut head_controller = HeadController::new(
        "/dev/ttyACM0",
        [30, 31],
        [PI / 2.0, PI / 2.0],
        [false, false],
        [1.0, 1.0],
        40,
    )?;

    loop {
        if let Ok(res) = head_controller.get_fan_state() {
            println!("FAN STATE {:?}", res);
        }

        if let Ok(res) = head_controller.set_fan_state([true, true]) {
            println!("FAN STATE {:?}", res);
        }
        thread::sleep(Duration::from_millis(3000));

        if let Ok(res) = head_controller.set_fan_state([false, true]) {
            println!("FAN STATE {:?}", res);
        }

        thread::sleep(Duration::from_millis(3000));

        if let Ok(res) = head_controller.set_fan_state([true, false]) {
            println!("FAN STATE {:?}", res);
        }

        thread::sleep(Duration::from_millis(3000));

        if let Ok(res) = head_controller.set_fan_state([false, false]) {
            println!("FAN STATE {:?}", res);
        }

        thread::sleep(Duration::from_millis(5000));
    }
}
