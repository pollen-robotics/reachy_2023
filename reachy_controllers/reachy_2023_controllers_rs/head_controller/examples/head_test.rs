use ::head_controller::HeadController;
use head_controller::PID;
use std::{
    error::Error,
    f64::consts::PI,
    thread,
    time::{Duration, SystemTime},
};

fn read_all(head_controller: &mut HeadController) -> Result<(), Box<dyn Error>> {
    // Read at every timestep
    // But in one shot
    let (_pos, _speed, _load) = head_controller.get_xl320_present_position_speed_load()?;

    // Read at about 1Hz?
    let _ = head_controller.get_xl320_temperature()?;

    // Read at about 1Hz?
    let _ = head_controller.get_xl320_pid()?;

    // Read at about 10Hz?
    let _ = head_controller.is_xl320_torque_on()?;

    // Read at about 1Hz?
    let _ = head_controller.get_fan_state()?;

    Ok(())
}

fn write_all(head_controller: &mut HeadController) -> Result<(), Box<dyn Error>> {
    head_controller.set_xl320_torque([false; 2])?;
    head_controller.set_xl320_target_position_speed_load([0.0; 2], [0.5; 2], [25.0; 2])?;

    let mut pids = vec![];
    for _ in 0..8 {
        pids.push(PID {
            p: 32.0,
            i: 0.0,
            d: 0.0,
        });
    }
    // ATTENTION !!!
    head_controller.set_xl320_pid(&pids)?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut head_controller = HeadController::new(
        "/dev/ttyACM0",
        [30, 31],
        [PI / 2.0, PI / 2.0],
        [false, false],
        [1.0, 1.0],
        30,
    )?;

    let mut i = 1;
    let mut read_error = 0;
    // let mut write_error = 0;

    loop {
        let t0 = SystemTime::now();
        if read_all(&mut head_controller).is_err() {
            read_error += 1;
        }
        let dt = t0.elapsed().unwrap();
        let e = read_error as f64 / i as f64 * 100.0;
        println!("READ {:.0}ms NB ERR: {:?}%", dt.as_secs_f32() * 1000.0, e);

        let t0 = SystemTime::now();
        if write_all(&mut head_controller).is_err() {
            //write_error += 1;
        }
        let dt = t0.elapsed().unwrap();
        //let e = write_error as f64 / i as f64 * 100.0;
        //println!("WRITE {:?}ms NB ERR: {:?}%", dt.as_secs_f32() * 1000.0, e);
        i += 1;
    }
}

// loop {
// xl320::sync_write_goal_position( & io, serial_port.as_mut(), & [11], & [2048]) ?;
//
// let temp = xl320::read_present_temperature( & io, serial_port.as_mut(), 11) ?;
// println ! ("{:?}", temp);
//
// thread::sleep(Duration::from_millis(500));
//
// xl320::sync_write_goal_position( &io, serial_port.as_mut(), & [11], & [1000]) ?;
// let pos = xl320::read_present_position( & io, serial_port.as_mut(), 11) ?;
// println ! ("{:?}", pos);
//
// thread::sleep(Duration::from_millis(500));
// }
// }
