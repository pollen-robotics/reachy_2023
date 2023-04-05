use cache_cache::Cache;
use itertools::{izip, multiunzip};
use motor_model::MotorModel;
use rustypot::{
    device::{l0_force_fan, mx},
    DynamixelSerialIO,
};
use serialport::SerialPort;
use std::{collections::HashMap, time::Duration};

pub struct ArmController {
    serial_port: Box<dyn SerialPort>,
    io: DynamixelSerialIO,

    mx_ids: [u8; 8],
    mx_params: HashMap<u8, MotorModel>,
    mx_present_temperature: Cache<u8, f64>,
    mx_pid: Cache<u8, PID>,
    mx_torque_on: Cache<u8, bool>,
    mx_goal_position: Cache<u8, f64>,
    mx_moving_speed: Cache<u8, f64>,
    mx_torque_limit: Cache<u8, f64>,

    force_sensor_id: u8,
    force_sensor: Cache<u8, i32>,

    fan_id: u8,
    fan_state: Cache<u8, [bool; 3]>,
}

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

impl ArmController {
    pub fn new(
        serial_port: &str,
        mx_ids: [u8; 8],
        offsets: [f64; 8],
        is_direct: [bool; 8],
        reductions: [f64; 8],
        force_sensor_id: u8,
        fan_id: u8,
    ) -> Result<Self> {
        let serial_port = serialport::new(serial_port, 1_000_000)
            .timeout(Duration::from_millis(10))
            .open()?;
        let io = DynamixelSerialIO::v1();

        let mx_params = izip!(mx_ids, offsets, is_direct, reductions)
            .map(|(id, offset, is_direct, reduction)| {
                (
                    id,
                    MotorModel {
                        offset,
                        is_direct,
                        reduction,
                    },
                )
            })
            .collect();

        Ok(ArmController {
            serial_port,
            io,

            mx_ids,
            mx_params,
            mx_present_temperature: Cache::with_expiry_duration(Duration::from_secs(1)),
            mx_pid: Cache::keep_last(),
            mx_torque_on: Cache::with_expiry_duration(Duration::from_secs(1)),
            mx_goal_position: Cache::keep_last(),
            mx_moving_speed: Cache::keep_last(),
            mx_torque_limit: Cache::keep_last(),

            fan_id,
            fan_state: Cache::keep_last(),

            force_sensor_id,
            force_sensor: Cache::with_expiry_duration(Duration::from_millis(100)),
        })
    }
    pub fn get_mx_present_position_speed_load(&mut self) -> Result<([f64; 8], [f64; 8], [f64; 8])> {
        let (pos, speed, load): (Vec<f64>, Vec<f64>, Vec<f64>) = multiunzip(
            mx::sync_read_present_position_speed_load(
                &self.io,
                self.serial_port.as_mut(),
                &self.mx_ids,
            )?
            .iter()
            .enumerate()
            .map(|(i, (p, s, l))| {
                let mx = &self.mx_params[&self.mx_ids[i]];

                (
                    mx.to_global_position(mx::conv::dxl_pos_to_radians(*p)),
                    mx.to_global_speed(mx::conv::dxl_oriented_speed_to_rad_per_sec(*s)),
                    mx.to_global_load(mx::conv::dxl_load_to_oriented_torque(*l)),
                )
            }),
        );

        Ok((
            pos.try_into().unwrap(),
            speed.try_into().unwrap(),
            load.try_into().unwrap(),
        ))
    }
    pub fn set_mx_target_position(&mut self, target_position: [f64; 8]) -> Result<()> {
        // Keep only the new goal position
        let current_target_pos = self.get_mx_target_position()?;
        let target_to_update = differences(&self.mx_ids, &current_target_pos, &target_position);

        // Keep only the goal position where the motor torque is actually on
        let torques = self.is_mx_torque_on()?;
        let torques: HashMap<&u8, bool> = self.mx_ids.iter().zip(torques).collect();
        let target_to_update: HashMap<&u8, &f64> = target_to_update
            .iter()
            .filter(|(id, _)| torques[id])
            .collect();

        if !target_to_update.is_empty() {
            // Convert USI value to dynamixel ones
            let (keys, values): (Vec<u8>, Vec<i16>) = target_to_update
                .iter()
                .map(|(&k, &v)| {
                    (
                        k,
                        mx::conv::radians_to_dxl_pos(self.mx_params[k].to_local_position(*v)),
                    )
                })
                .unzip();

            // Send new values to the motors
            mx::sync_write_goal_position(&self.io, self.serial_port.as_mut(), &keys, &values)?;
            // Update our cache
            for (k, v) in target_to_update {
                self.mx_goal_position.insert(*k, *v);
            }
        }
        Ok(())
    }
    pub fn set_mx_moving_speed(&mut self, moving_speed: [f64; 8]) -> Result<()> {
        let current_moving_speed = self.get_mx_moving_speed()?;
        let moving_speed_to_update =
            differences(&self.mx_ids, &current_moving_speed, &moving_speed);
        if !moving_speed_to_update.is_empty() {
            let mut err = None;

            for (k, v) in moving_speed_to_update {
                let dxl_v = mx::conv::rad_per_sec_to_dxl_abs_speed(
                    self.mx_params[&k].to_local_max_speed(v),
                );

                match mx::write_moving_speed(&self.io, self.serial_port.as_mut(), k, dxl_v) {
                    Ok(_) => {
                        self.mx_moving_speed.insert(k, v);
                    }
                    Err(e) => {
                        err = Some(e);
                    }
                }
            }

            if let Some(err) = err {
                return Err(err);
            }
        }
        Ok(())
    }
    pub fn set_mx_torque_limit(&mut self, torque_limit: [f64; 8]) -> Result<()> {
        let current_torque_limit = self.get_mx_torque_limit()?;
        let torque_limit_to_update =
            differences(&self.mx_ids, &current_torque_limit, &torque_limit);
        if !torque_limit_to_update.is_empty() {
            let mut err = None;

            for (k, v) in torque_limit_to_update {
                let dxl_t = mx::conv::torque_to_dxl_abs_load(v);

                match mx::write_torque_limit(&self.io, self.serial_port.as_mut(), k, dxl_t) {
                    Ok(_) => {
                        self.mx_torque_limit.insert(k, v);
                    }
                    Err(e) => {
                        err = Some(e);
                    }
                }
            }

            if let Some(err) = err {
                return Err(err);
            }
        }

        Ok(())
    }
    pub fn set_mx_target_position_speed_load(
        &mut self,
        target_position: [f64; 8],
        moving_speed: [f64; 8],
        torque_limit: [f64; 8],
    ) -> Result<()> {
        self.set_mx_moving_speed(moving_speed)?;
        self.set_mx_torque_limit(torque_limit)?;
        self.set_mx_target_position(target_position)?;
        Ok(())
    }
    pub fn get_mx_temperature(&mut self) -> Result<[f64; 8]> {
        Ok(self
            .mx_present_temperature
            .entries(&self.mx_ids)
            .or_try_insert_with(|ids| {
                Ok(
                    mx::sync_read_present_temperature(&self.io, self.serial_port.as_mut(), ids)?
                        .iter()
                        .map(|&t| t as f64)
                        .collect(),
                )
            })?
            .as_slice()
            .try_into()
            .unwrap())
    }
    pub fn is_mx_torque_on(&mut self) -> Result<[bool; 8]> {
        Ok(self
            .mx_torque_on
            .entries(&self.mx_ids)
            .or_try_insert_with(|ids| {
                Ok(
                    mx::sync_read_torque_enable(&self.io, self.serial_port.as_mut(), ids)?
                        .iter()
                        .map(|&t| t != 0)
                        .collect(),
                )
            })?
            .as_slice()
            .try_into()
            .unwrap())
    }
    pub fn set_mx_torque(&mut self, torque: [bool; 8]) -> Result<()> {
        let current_torque = self.is_mx_torque_on()?;
        let torque_to_update = differences(&self.mx_ids, &current_torque, &torque);

        if !torque_to_update.is_empty() {
            let mut err = None;

            for (k, v) in torque_to_update {
                match mx::write_torque_enable(&self.io, self.serial_port.as_mut(), k, v as u8) {
                    Ok(_) => {
                        self.mx_torque_on.insert(k, v);
                    }
                    Err(e) => {
                        err = Some(e);
                    }
                };
            }

            if let Some(err) = err {
                return Err(err);
            }
        }

        Ok(())
    }
    pub fn get_mx_pid(&mut self) -> Result<Vec<PID>> {
        self.mx_pid.entries(&self.mx_ids).or_try_insert_with(|ids| {
            let mx_p = mx::sync_read_p_gain(&self.io, self.serial_port.as_mut(), ids)?;
            let mx_i = mx::sync_read_i_gain(&self.io, self.serial_port.as_mut(), ids)?;
            let mx_d = mx::sync_read_d_gain(&self.io, self.serial_port.as_mut(), ids)?;

            Ok(izip!(mx_p, mx_i, mx_d)
                .map(|(p, i, d)| PID {
                    p: p as f64,
                    i: i as f64,
                    d: d as f64,
                })
                .collect())
        })
    }
    pub fn set_mx_pid(&mut self, pid: &Vec<PID>) -> Result<()> {
        assert_eq!(pid.len(), 8);

        let current_pid = self.get_mx_pid()?;
        let pid_to_update = differences(&self.mx_ids, &current_pid, pid);

        if !pid_to_update.is_empty() {
            let mut err = None;

            for (k, v) in pid_to_update {
                let mut is_ok = true;

                if let Err(e) = mx::write_p_gain(&self.io, self.serial_port.as_mut(), k, v.p as u8)
                {
                    err = Some(e);
                    is_ok = false;
                }
                if let Err(e) = mx::write_i_gain(&self.io, self.serial_port.as_mut(), k, v.i as u8)
                {
                    err = Some(e);
                    is_ok = false;
                }
                if let Err(e) = mx::write_d_gain(&self.io, self.serial_port.as_mut(), k, v.d as u8)
                {
                    err = Some(e);
                    is_ok = false;
                }

                if is_ok {
                    self.mx_pid.insert(k, v);
                }
            }

            if let Some(err) = err {
                return Err(err);
            }
        }

        Ok(())
    }
    pub fn get_mx_target_position(&mut self) -> Result<[f64; 8]> {
        Ok(self
            .mx_goal_position
            .entries(&self.mx_ids)
            .or_try_insert_with(|ids| {
                Ok(
                    mx::sync_read_goal_position(&self.io, self.serial_port.as_mut(), ids)?
                        .iter()
                        .enumerate()
                        .map(|(i, &p)| {
                            self.mx_params[&ids[i]]
                                .to_global_position(mx::conv::dxl_pos_to_radians(p))
                        })
                        .collect::<Vec<f64>>(),
                )
            })?
            .as_slice()
            .try_into()
            .unwrap())
    }
    pub fn get_mx_moving_speed(&mut self) -> Result<[f64; 8]> {
        Ok(self
            .mx_moving_speed
            .entries(&self.mx_ids)
            .or_try_insert_with(|ids| {
                Ok(
                    mx::sync_read_moving_speed(&self.io, self.serial_port.as_mut(), ids)?
                        .iter()
                        .enumerate()
                        .map(|(i, &s)| {
                            self.mx_params[&ids[i]]
                                .to_global_max_speed(mx::conv::dxl_abs_speed_to_rad_per_sec(s))
                        })
                        .collect::<Vec<f64>>(),
                )
            })?
            .as_slice()
            .try_into()
            .unwrap())
    }
    pub fn get_mx_torque_limit(&mut self) -> Result<[f64; 8]> {
        Ok(self
            .mx_torque_limit
            .entries(&self.mx_ids)
            .or_try_insert_with(|ids| {
                Ok(
                    mx::sync_read_torque_limit(&self.io, self.serial_port.as_mut(), ids)?
                        .iter()
                        .map(|&l| mx::conv::dxl_load_to_abs_torque(l))
                        .collect::<Vec<f64>>(),
                )
            })?
            .as_slice()
            .try_into()
            .unwrap())
    }
    pub fn read_force_sensor(&mut self) -> Result<f64> {
        Ok(self
            .force_sensor
            .entry(self.force_sensor_id)
            .or_try_insert_with(|&id| {
                l0_force_fan::read_present_load(&self.io, self.serial_port.as_mut(), id)
            })? as f64)
    }
    pub fn get_fan_state(&mut self) -> Result<[bool; 3]> {
        self.fan_state.entry(self.fan_id).or_try_insert_with(|&id| {
            Ok([
                l0_force_fan::read_fan1_state(&self.io, self.serial_port.as_mut(), id)? != 0,
                l0_force_fan::read_fan2_state(&self.io, self.serial_port.as_mut(), id)? != 0,
                l0_force_fan::read_fan3_state(&self.io, self.serial_port.as_mut(), id)? != 0,
            ])
        })
    }
    pub fn set_fan_state(&mut self, state: [bool; 3]) -> Result<()> {
        let current_state = self.get_fan_state()?;
        let mut need_update = false;

        if state[0] != current_state[0] {
            l0_force_fan::write_fan1_state(
                &self.io,
                self.serial_port.as_mut(),
                self.fan_id,
                state[0] as u8,
            )?;
            need_update = true;
        }

        if state[1] != current_state[1] {
            l0_force_fan::write_fan2_state(
                &self.io,
                self.serial_port.as_mut(),
                self.fan_id,
                state[1] as u8,
            )?;
            need_update = true;
        }
        if state[2] != current_state[2] {
            l0_force_fan::write_fan3_state(
                &self.io,
                self.serial_port.as_mut(),
                self.fan_id,
                state[2] as u8,
            )?;
            need_update = true;
        }
        if need_update {
            self.fan_state.insert(self.fan_id, state);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialOrd)]
pub struct PID {
    pub p: f64,
    pub i: f64,
    pub d: f64,
}

impl PartialEq for PID {
    fn eq(&self, other: &Self) -> bool {
        self.p == other.p && self.i == other.i && self.d == other.d
    }
}

fn differences<V: PartialOrd + Copy>(
    ids: &[u8],
    old_value: &[V],
    new_value: &[V],
) -> HashMap<u8, V> {
    izip!(ids, old_value, new_value)
        .filter(|(_, ov, nv)| ov != nv)
        .map(|(id, _, nv)| (*id, *nv))
        .collect()
}
