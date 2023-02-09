use ::arm_controller::ArmController;
use arm_controller::PID;
use std::{
    error::Error,
    f64::consts::PI,
    thread,
    time::{Duration, SystemTime},
};

fn read_all(arm_controller: &mut ArmController) -> Result<(), Box<dyn Error>> {
    // Read at every timestep
    // But in one shot
    let (_pos, _speed, _load) = arm_controller.get_mx_present_position_speed_load()?;

    // Read at about 1Hz?
    let _ = arm_controller.get_mx_temperature()?;

    // Read at about 1Hz?
    let _ = arm_controller.get_mx_pid()?;

    // Read at about 10Hz?
    let _ = arm_controller.is_mx_torque_on()?;

    // Read at about 10Hz?
    let _ = arm_controller.read_force_sensor()?;

    // Read at about 1Hz?
    let _ = arm_controller.get_fan_state()?;

    Ok(())
}

fn write_all(arm_controller: &mut ArmController) -> Result<(), Box<dyn Error>> {
    arm_controller.set_mx_torque([false; 8])?;
    arm_controller.set_mx_target_position_speed_load([0.0; 8], [0.5; 8], [25.0; 8])?;

    let mut pids = vec![];
    for _ in 0..8 {
        pids.push(PID {
            p: 32.0,
            i: 0.0,
            d: 0.0,
        });
    }
    // ATTENTION !!!
    arm_controller.set_mx_pid(&pids)?;
    arm_controller.set_fan_state([false; 3])?;

    Ok(())
}

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
        let t0 = SystemTime::now();
        if read_all(&mut arm_controller).is_err() {
            read_error += 1;
        }
        let dt = t0.elapsed().unwrap();
        let e = read_error as f64 / i as f64 * 100.0;
        println!("READ {:.0}ms NB ERR: {:?}%", dt.as_secs_f32() * 1000.0, e);

        // let t0 = SystemTime::now();
        // if write_all(&mut arm_controller).is_err() {
        //     write_error += 1;
        // }
        // let dt = t0.elapsed().unwrap();
        // let e = write_error as f64 / i as f64 * 100.0;
        // println!("WRITE {:?}ms NB ERR: {:?}%", dt.as_secs_f32() * 1000.0, e);

        i += 1;
    }
}
