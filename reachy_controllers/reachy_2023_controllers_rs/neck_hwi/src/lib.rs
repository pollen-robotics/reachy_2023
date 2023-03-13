use std::{collections::HashMap, ffi::CStr, sync::Mutex};

use orbita_serial_controller::{OrbitaController, Orientation};
use rustypot::device::orbita_foc::{DiskValue, Pid};

#[macro_use]
extern crate lazy_static;

lazy_static! {
    static ref UID: Mutex<u32> = Mutex::new(0);
    static ref NECK_CONTROLLER: Mutex<HashMap<u32, OrbitaController>> = Mutex::new(HashMap::new());
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn neck_hwi_init(
    serial_port: *const libc::c_char,
    id: u8,
    alpha: f64,
    hardware_zero: *mut f64,
    reduction: f64,
) -> u32 {
    let serial_port = unsafe { CStr::from_ptr(serial_port) }.to_str().unwrap();
    let hardware_zero = unsafe { std::slice::from_raw_parts_mut(hardware_zero, 3) };
    let hardware_zero = DiskValue {
        top: hardware_zero[0] as f32,
        middle: hardware_zero[1] as f32,
        bottom: hardware_zero[2] as f32,
    };

    let c = OrbitaController::with_startup_position_as_approximate_hardware_zero(
        serial_port,
        id,
        alpha,
        hardware_zero,
        reduction as f32,
    )
    .unwrap();

    let uid = get_available_uid();

    NECK_CONTROLLER.lock().unwrap().insert(uid, c);

    uid
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn neck_hwi_get_orientation(uid: u32, orientation: *mut f64) -> i32 {
    let orientation = unsafe { std::slice::from_raw_parts_mut(orientation, 3) };

    match NECK_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_current_rpy_orientation()
    {
        Ok(o) => {
            orientation[0] = o.roll;
            orientation[1] = o.pitch;
            orientation[2] = o.yaw;

            0
        }
        Err(_) => 1,
    }
}

// #[no_mangle]
// #[allow(clippy::not_unsafe_ptr_arg_deref)]
// pub extern "C" fn neck_hwi_get_orientation_velocity_load(
//     uid: u32,
//     orientation: *mut f64,
//     velocity: *mut f64,
//     load: *mut f64,
// ) -> i32 {
//     let orientation = unsafe { std::slice::from_raw_parts_mut(orientation, 3) };
//     let velocity = unsafe { std::slice::from_raw_parts_mut(velocity, 3) };
//     let load = unsafe { std::slice::from_raw_parts_mut(load, 3) };

//     match NECK_CONTROLLER
//         .lock()
//         .unwrap()
//         .get_mut(&uid)
//         .unwrap()
//         .get_current_rpy_orientation_velocity_effort()
//     {
//         Ok((o, v, l)) => {
//             orientation[0] = o.roll;
//             orientation[1] = o.pitch;
//             orientation[2] = o.yaw;

//             velocity[0] = v.roll;
//             velocity[1] = v.pitch;
//             velocity[2] = v.yaw;

//             load[0] = l.roll;
//             load[1] = l.pitch;
//             load[2] = l.yaw;

//             0
//         }
//         Err(_) => 1,
//     }
// }

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn neck_hwi_get_goal_orientation(uid: u32, target_orientation: *mut f64) -> i32 {
    let target_orientation = unsafe { std::slice::from_raw_parts_mut(target_orientation, 3) };

    match NECK_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_target_rpy_orientation()
    {
        Ok(o) => {
            target_orientation[0] = o.roll;
            target_orientation[1] = o.pitch;
            target_orientation[2] = o.yaw;

            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
// pub extern "C" fn neck_hwi_set_target_orientation_max_speed_max_torque(
pub extern "C" fn neck_hwi_set_target_orientation_max_speed(
    uid: u32,
    target_orientation: *mut f64,
    speed_limit: *mut f64,
    // torque_limit: *mut f64,
) -> i32 {
    let target_orientation = unsafe { std::slice::from_raw_parts_mut(target_orientation, 3) };
    let target_orientation = Orientation {
        roll: target_orientation[0],
        pitch: target_orientation[1],
        yaw: target_orientation[2],
    };

    // FIXME: This should not be done on each joint.
    let speed_limit = unsafe { std::slice::from_raw_parts_mut(speed_limit, 3) };
    let speed_limit = speed_limit[0];

    // let torque_limit = unsafe { std::slice::from_raw_parts_mut(torque_limit, 3) };
    // let torque_limit = torque_limit[0];

    if NECK_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_target_rpy_orientation(target_orientation)
        .is_err()
    {
        return 1;
    }

    if NECK_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_max_speed(speed_limit as f32)
        .is_err()
    {
        return 1;
    }

    // if NECK_CONTROLLER
    //     .lock()
    //     .unwrap()
    //     .get_mut(&uid)
    //     .unwrap()
    //     .set_max_torque(torque_limit as f32)
    //     .is_err()
    // {
    //     return 1;
    // }

    0
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn neck_hwi_get_max_speed(uid: u32, max_speed: *mut f64) -> i32 {
    let max_speed = unsafe { std::slice::from_raw_parts_mut(max_speed, 3) };

    match NECK_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_max_speed()
    {
        Ok(s) => {
            for i in 0..3 {
                max_speed[i] = s as f64;
            }
            0
        }
        Err(_) => 1,
    }
}

// #[no_mangle]
// #[allow(clippy::not_unsafe_ptr_arg_deref)]
// pub extern "C" fn neck_hwi_get_max_torque(uid: u32, max_torque: *mut f64) -> i32 {
//     let max_torque = unsafe { std::slice::from_raw_parts_mut(max_torque, 3) };

//     for i in 0..3 {
//         max_torque[i] = 42.0;
//     }
//     0

//     // match NECK_CONTROLLER
//     //     .lock()
//     //     .unwrap()
//     //     .get_mut(&uid)
//     //     .unwrap()
//     //     .get_max_torque()
//     // {
//     //     Ok(t) => {
//     //         for i in 0..3 {
//     //             max_torque[i] = t as f64;
//     //         }
//     //         0
//     //     }
//     //     Err(_) => 1,
//     // }
// }

// #[no_mangle]
// #[allow(clippy::not_unsafe_ptr_arg_deref)]
// pub extern "C" fn neck_hwi_get_temperature(uid: u32, temperature: *mut f64) -> i32 {
//     let temperature = unsafe { std::slice::from_raw_parts_mut(temperature, 3) };

//     match NECK_CONTROLLER
//         .lock()
//         .unwrap()
//         .get_mut(&uid)
//         .unwrap()
//         .get_motor_temperature()
//     {
//         Ok(t) => {
//             temperature[0] = t.top as f64;
//             temperature[1] = t.middle as f64;
//             temperature[2] = t.bottom as f64;

//             0
//         }
//         Err(_) => 1,
//     }
// }

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn neck_hwi_is_torque_on(uid: u32, is_on: *mut f64) -> i32 {
    let is_on = unsafe { std::slice::from_raw_parts_mut(is_on, 3) };

    match NECK_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .is_torque_on()
    {
        Ok(t) => {
            for i in 0..3 {
                is_on[i] = if t { 1.0 } else { 0.0 };
            }
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn neck_hwi_set_torque(uid: u32, on: *mut f64) -> i32 {
    let on = unsafe { std::slice::from_raw_parts_mut(on, 3) };

    // FIXME: This should not be done on each joint.
    let on = on[0] != 0.0;

    if on {
        match NECK_CONTROLLER
            .lock()
            .unwrap()
            .get_mut(&uid)
            .unwrap()
            .enable_torque()
        {
            Ok(_) => 0,
            Err(_) => 1,
        }
    } else {
        match NECK_CONTROLLER
            .lock()
            .unwrap()
            .get_mut(&uid)
            .unwrap()
            .disable_torque()
        {
            Ok(_) => 0,
            Err(_) => 1,
        }
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn neck_hwi_get_pid(uid: u32, p: *mut f64, i: *mut f64, d: *mut f64) -> i32 {
    let p = unsafe { std::slice::from_raw_parts_mut(p, 3) };
    let i = unsafe { std::slice::from_raw_parts_mut(i, 3) };
    let d = unsafe { std::slice::from_raw_parts_mut(d, 3) };

    match NECK_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_angle_pid()
    {
        Ok(pid) => {
            for j in 0..3 {
                p[j] = pid.p as f64;
                i[j] = pid.i as f64;
                d[j] = pid.d as f64;
            }

            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn neck_hwi_set_pid(uid: u32, p: *mut f64, i: *mut f64, d: *mut f64) -> i32 {
    let p = unsafe { std::slice::from_raw_parts_mut(p, 3) };
    let i = unsafe { std::slice::from_raw_parts_mut(i, 3) };
    let d = unsafe { std::slice::from_raw_parts_mut(d, 3) };

    // FIXME: This should not be done for each joint!
    let pid = Pid {
        p: p[0] as f32,
        i: i[0] as f32,
        d: d[0] as f32,
    };

    match NECK_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_angle_pid(pid)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

fn get_available_uid() -> u32 {
    let mut uid = UID.lock().unwrap();

    *uid += 1;
    *uid
}
