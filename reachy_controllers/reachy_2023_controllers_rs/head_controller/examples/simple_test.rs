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
        30,
    )?;

    // let mut pid_test:Vec<PID> = vec![];
    // pid_test.push(PID{p:47.0, i:1.1,d:2.0});
    // pid_test.push(PID{p:32.0, i:0.0,d:2.5});
    // if let Ok(res) = head_controller.set_xl320_pid(pid_test) {
    //     println!("WRITE {:?}", res);
    // }
    //
    // if let Ok(res) = head_controller.get_xl320_pid() {
    //     println!("READ {:?}", res);
    // }

    // if let Ok(res) = head_controller.is_xl320_torque_on() {
    //     println!("READ {:?}", res);
    // }

    // if let Ok(res) = head_controller.set_xl320_torque([true, true]) {
    //     println!("WRITE {:?}", res);
    // }

    // if let Ok(res) = head_controller.set_xl320_torque_limit([30.0,30.0]) {
    //     println!("READ {:?}", res);
    // }
    // if let Ok(res) = head_controller.set_xl320_moving_speed([10.0, 20.0]) {
    //     println!("READ {:?}", res);
    // }

    // if let Ok(res) = head_controller.is_xl320_torque_on() {
    //     println!("READ {:?}", res);
    // }

    // if let Ok(res) = head_controller.get_xl320_moving_speed() {
    //     println!("READ {:?}", res);
    // }

    loop {
        // head_controller.set_xl320_target_position_speed_load([0.0, 1.0], [100.0, 100.0], [100.0, 100.0])?;
        thread::sleep(Duration::from_millis(1000));

        // head_controller.set_xl320_target_position_speed_load([1.0, 0.0], [100.0, 100.0], [100.0, 100.0])?;
        // if let Ok(res) = head_controller.get_xl320_present_position_speed_load() {
        //     println!("{:?}", res);
        // }
        // if let Ok(res) = head_controller.get_xl320_temperature() {
        //     println!("{:?}", res);
        // }
        // if let Ok(res) = head_controller.get_xl320_pid() {
        //     println!("{:?}", res);
        // }

        thread::sleep(Duration::from_millis(1000));
        if let Ok(res) = head_controller.get_xl320_moving_speed() {
            println!("READ {:?}", res);
        }
        if let Ok(res) = head_controller.get_xl320_torque_limit() {
            println!("READ {:?}", res);
        }
    }
}
