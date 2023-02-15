use std::{collections::HashMap, ffi::CStr, sync::Mutex};

use head_controller::{HeadController, PID};
use itertools::izip;

#[macro_use]
extern crate lazy_static;

lazy_static! {
    static ref UID: Mutex<u32> = Mutex::new(0);
    static ref HEAD_CONTROLLER: Mutex<HashMap<u32, HeadController>> = Mutex::new(HashMap::new());
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn head_hwi_init(
    serial_port: *const libc::c_char,
    xl320_ids: *mut u8,
    offsets: *mut f64,
    is_direct: *mut bool,
    reductions: *mut f64,
    fan_id: u8,
) -> u32 {
    let serial_port = unsafe { CStr::from_ptr(serial_port) }.to_str().unwrap();
    let xl320_ids = unsafe { std::slice::from_raw_parts_mut(xl320_ids, 2) }
        .try_into()
        .unwrap();
    let offsets = unsafe { std::slice::from_raw_parts_mut(offsets, 2) }
        .try_into()
        .unwrap();
    let is_direct = unsafe { std::slice::from_raw_parts_mut(is_direct, 2) }
        .try_into()
        .unwrap();
    let reductions = unsafe { std::slice::from_raw_parts_mut(reductions, 2) }
        .try_into()
        .unwrap();

    let c = HeadController::new(
        serial_port,
        xl320_ids,
        offsets,
        is_direct,
        reductions,
        fan_id,
    )
    .unwrap();

    let uid = get_available_uid();

    HEAD_CONTROLLER.lock().unwrap().insert(uid, c);

    uid
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn head_hwi_get_xl320_present_position_speed_load(
    uid: u32,
    position: *mut f64,
    speed: *mut f64,
    load: *mut f64,
) -> i32 {
    let position = unsafe { std::slice::from_raw_parts_mut(position, 2) };
    let speed = unsafe { std::slice::from_raw_parts_mut(speed, 2) };
    let load = unsafe { std::slice::from_raw_parts_mut(load, 2) };

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_xl320_present_position_speed_load()
    {
        Ok((p, s, l)) => {
            position[..2].copy_from_slice(&p[..2]);
            speed[..2].copy_from_slice(&s[..2]);
            load[..2].copy_from_slice(&l[..2]);

            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn head_hwi_get_goal_position(uid: u32, goal_position: *mut f64) -> i32 {
    let goal_position = unsafe { std::slice::from_raw_parts_mut(goal_position, 2) };

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_xl320_target_position()
    {
        Ok(gp) => {
            goal_position[..2].copy_from_slice(&gp[..2]);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn head_hwi_get_moving_speed(uid: u32, moving_speed: *mut f64) -> i32 {
    let moving_speed = unsafe { std::slice::from_raw_parts_mut(moving_speed, 2) };

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_xl320_moving_speed()
    {
        Ok(ms) => {
            moving_speed[..2].copy_from_slice(&ms[..2]);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn head_hwi_get_torque_limit(uid: u32, torque_limit: *mut f64) -> i32 {
    let torque_limit = unsafe { std::slice::from_raw_parts_mut(torque_limit, 2) };

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_xl320_torque_limit()
    {
        Ok(tl) => {
            torque_limit[..2].copy_from_slice(&tl[..2]);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn head_hwi_set_xl320_target_position_speed_load(
    uid: u32,
    target_position: *mut f64,
    moving_speed: *mut f64,
    torque_limit: *mut f64,
) -> i32 {
    let target_position = unsafe { std::slice::from_raw_parts_mut(target_position, 2) };
    let moving_speed = unsafe { std::slice::from_raw_parts_mut(moving_speed, 2) };
    let torque_limit = unsafe { std::slice::from_raw_parts_mut(torque_limit, 2) };

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_xl320_target_position_speed_load(
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
pub extern "C" fn head_hwi_set_xl320_torque_limit(uid: u32, torque_limit: *mut f64) -> i32 {
    let torque_limit = unsafe { std::slice::from_raw_parts_mut(torque_limit, 2) };

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_xl320_torque_limit(torque_limit.try_into().unwrap())
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn head_hwi_set_xl320_speed_limit(uid: u32, speed_limit: *mut f64) -> i32 {
    let speed_limit = unsafe { std::slice::from_raw_parts_mut(speed_limit, 2) };

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_xl320_moving_speed(speed_limit.try_into().unwrap())
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn head_hwi_get_xl320_temperature(uid: u32, temperature: *mut f64) -> i32 {
    let temperature = unsafe { std::slice::from_raw_parts_mut(temperature, 2) };

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_xl320_temperature()
    {
        Ok(t) => {
            temperature[..2].copy_from_slice(&t[..2]);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn head_hwi_is_xl320_torque_on(uid: u32, is_on: *mut f64) -> i32 {
    let is_on = unsafe { std::slice::from_raw_parts_mut(is_on, 2) };

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .is_xl320_torque_on()
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
pub extern "C" fn head_hwi_set_xl320_torque(uid: u32, on: *mut f64) -> i32 {
    let on = unsafe { std::slice::from_raw_parts_mut(on, 2) };
    let torque = on.iter().map(|&on| on != 0.0).collect::<Vec<bool>>();

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_xl320_torque(torque.as_slice().try_into().unwrap())
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn head_hwi_get_xl320_pid(uid: u32, p: *mut f64, i: *mut f64, d: *mut f64) -> i32 {
    let p = unsafe { std::slice::from_raw_parts_mut(p, 2) };
    let i = unsafe { std::slice::from_raw_parts_mut(i, 2) };
    let d = unsafe { std::slice::from_raw_parts_mut(d, 2) };

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_xl320_pid()
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
pub extern "C" fn head_hwi_set_xl320_pid(uid: u32, p: *mut f64, i: *mut f64, d: *mut f64) -> i32 {
    let p = unsafe { std::slice::from_raw_parts_mut(p, 2) };
    let i = unsafe { std::slice::from_raw_parts_mut(i, 2) };
    let d = unsafe { std::slice::from_raw_parts_mut(d, 2) };

    let pid = izip!(p, i, d)
        .map(|(p, i, d)| PID {
            p: *p,
            i: *i,
            d: *d,
        })
        .collect::<Vec<PID>>();

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_xl320_pid(&pid)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn head_hwi_get_fan_state(uid: u32, fan_state: *mut f64) -> i32 {
    let fan_state = unsafe { std::slice::from_raw_parts_mut(fan_state, 2) };

    match HEAD_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_fan_state()
    {
        Ok(s) => {
            fan_state[..2].copy_from_slice(
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
pub extern "C" fn head_hwi_set_fan_state(uid: u32, fan_state: *mut f64) -> i32 {
    let fan_state = unsafe { std::slice::from_raw_parts_mut(fan_state, 2) };
    let fan_state = fan_state.iter().map(|&s| s != 0.0).collect::<Vec<bool>>();

    match HEAD_CONTROLLER
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
