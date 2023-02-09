use std::{collections::HashMap, ffi::CStr, sync::Mutex};

use arm_controller::{ArmController, PID};
use itertools::izip;

#[macro_use]
extern crate lazy_static;

lazy_static! {
    static ref UID: Mutex<u32> = Mutex::new(0);
    static ref ARM_CONTROLLER: Mutex<HashMap<u32, ArmController>> = Mutex::new(HashMap::new());
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_init(
    serial_port: *const libc::c_char,
    mx_ids: *mut u8,
    offsets: *mut f64,
    is_direct: *mut bool,
    reductions: *mut f64,
    fan_id: u8,
    force_sensor_id: u8,
) -> u32 {
    let serial_port = unsafe { CStr::from_ptr(serial_port) }.to_str().unwrap();
    let mx_ids = unsafe { std::slice::from_raw_parts_mut(mx_ids, 8) }
        .try_into()
        .unwrap();
    let offsets = unsafe { std::slice::from_raw_parts_mut(offsets, 8) }
        .try_into()
        .unwrap();
    let is_direct = unsafe { std::slice::from_raw_parts_mut(is_direct, 8) }
        .try_into()
        .unwrap();
    let reductions = unsafe { std::slice::from_raw_parts_mut(reductions, 8) }
        .try_into()
        .unwrap();

    let c = ArmController::new(
        serial_port,
        mx_ids,
        offsets,
        is_direct,
        reductions,
        force_sensor_id,
        fan_id,
    )
    .unwrap();

    let uid = get_available_uid();

    ARM_CONTROLLER.lock().unwrap().insert(uid, c);

    uid
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_get_mx_present_position_speed_load(
    uid: u32,
    position: *mut f64,
    speed: *mut f64,
    load: *mut f64,
) -> i32 {
    let position = unsafe { std::slice::from_raw_parts_mut(position, 8) };
    let speed = unsafe { std::slice::from_raw_parts_mut(speed, 8) };
    let load = unsafe { std::slice::from_raw_parts_mut(load, 8) };

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_mx_present_position_speed_load()
    {
        Ok((p, s, l)) => {
            position[..8].copy_from_slice(&p[..8]);
            speed[..8].copy_from_slice(&s[..8]);
            load[..8].copy_from_slice(&l[..8]);

            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_get_goal_position(uid: u32, goal_position: *mut f64) -> i32 {
    let goal_position = unsafe { std::slice::from_raw_parts_mut(goal_position, 8) };

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_mx_target_position()
    {
        Ok(gp) => {
            goal_position[..8].copy_from_slice(&gp[..8]);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_get_moving_speed(uid: u32, moving_speed: *mut f64) -> i32 {
    let moving_speed = unsafe { std::slice::from_raw_parts_mut(moving_speed, 8) };

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_mx_moving_speed()
    {
        Ok(ms) => {
            moving_speed[..8].copy_from_slice(&ms[..8]);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_get_torque_limit(uid: u32, torque_limit: *mut f64) -> i32 {
    let torque_limit = unsafe { std::slice::from_raw_parts_mut(torque_limit, 8) };

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_mx_torque_limit()
    {
        Ok(tl) => {
            torque_limit[..8].copy_from_slice(&tl[..8]);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_set_mx_target_position_speed_load(
    uid: u32,
    target_position: *mut f64,
    moving_speed: *mut f64,
    torque_limit: *mut f64,
) -> i32 {
    let target_position = unsafe { std::slice::from_raw_parts_mut(target_position, 8) };
    let moving_speed = unsafe { std::slice::from_raw_parts_mut(moving_speed, 8) };
    let torque_limit = unsafe { std::slice::from_raw_parts_mut(torque_limit, 8) };

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_mx_target_position_speed_load(
            target_position.try_into().unwrap(),
            moving_speed.try_into().unwrap(),
            torque_limit.try_into().unwrap(),
        ) {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_set_mx_torque_limit(uid: u32, torque_limit: *mut f64) -> i32 {
    let torque_limit = unsafe { std::slice::from_raw_parts_mut(torque_limit, 8) };

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_mx_torque_limit(torque_limit.try_into().unwrap())
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_set_mx_speed_limit(uid: u32, speed_limit: *mut f64) -> i32 {
    let speed_limit = unsafe { std::slice::from_raw_parts_mut(speed_limit, 8) };

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_mx_moving_speed(speed_limit.try_into().unwrap())
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_get_mx_temperature(uid: u32, temperature: *mut f64) -> i32 {
    let temperature = unsafe { std::slice::from_raw_parts_mut(temperature, 8) };

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_mx_temperature()
    {
        Ok(t) => {
            temperature[..8].copy_from_slice(&t[..8]);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_is_mx_torque_on(uid: u32, is_on: *mut f64) -> i32 {
    let is_on = unsafe { std::slice::from_raw_parts_mut(is_on, 8) };

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .is_mx_torque_on()
    {
        Ok(o) => {
            for (i, on) in o.iter().enumerate() {
                is_on[i] = if *on { 1.0 } else { 0.0 };
            }
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_set_mx_torque(uid: u32, on: *mut f64) -> i32 {
    let on = unsafe { std::slice::from_raw_parts_mut(on, 8) };
    let torque = on.iter().map(|&on| on != 0.0).collect::<Vec<bool>>();

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_mx_torque(torque.as_slice().try_into().unwrap())
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_get_mx_pid(uid: u32, p: *mut f64, i: *mut f64, d: *mut f64) -> i32 {
    let p = unsafe { std::slice::from_raw_parts_mut(p, 8) };
    let i = unsafe { std::slice::from_raw_parts_mut(i, 8) };
    let d = unsafe { std::slice::from_raw_parts_mut(d, 8) };

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_mx_pid()
    {
        Ok(pids) => {
            for (n, pid) in pids.iter().enumerate() {
                p[n] = pid.p;
                i[n] = pid.i;
                d[n] = pid.d;
            }
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_set_mx_pid(uid: u32, p: *mut f64, i: *mut f64, d: *mut f64) -> i32 {
    let p = unsafe { std::slice::from_raw_parts_mut(p, 8) };
    let i = unsafe { std::slice::from_raw_parts_mut(i, 8) };
    let d = unsafe { std::slice::from_raw_parts_mut(d, 8) };

    let pid = izip!(p, i, d)
        .map(|(p, i, d)| PID {
            p: *p,
            i: *i,
            d: *d,
        })
        .collect::<Vec<PID>>();

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_mx_pid(&pid)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_read_force_sensor(uid: u32, force: &mut f64) -> i32 {
    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .read_force_sensor()
    {
        Ok(f) => {
            *force = f;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_get_fan_state(uid: u32, fan_state: *mut f64) -> i32 {
    let fan_state = unsafe { std::slice::from_raw_parts_mut(fan_state, 3) };

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_fan_state()
    {
        Ok(s) => {
            fan_state[..3].copy_from_slice(
                s.iter()
                    .map(|&s| if s { 1.0 } else { 0.0 })
                    .collect::<Vec<f64>>()
                    .as_slice(),
            );
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn arm_hwi_set_fan_state(uid: u32, fan_state: *mut f64) -> i32 {
    let fan_state = unsafe { std::slice::from_raw_parts_mut(fan_state, 3) };
    let fan_state = fan_state.iter().map(|&s| s != 0.0).collect::<Vec<bool>>();

    match ARM_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_fan_state(fan_state.try_into().unwrap())
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
