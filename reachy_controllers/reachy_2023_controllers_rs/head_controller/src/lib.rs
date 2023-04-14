use cache_cache::Cache;
use itertools::{izip, multiunzip};
use motor_model::MotorModel;
use rustypot::{
    device::{l0_force_fan, xl320},
    DynamixelSerialIO,
};
use serialport::SerialPort;
use std::{collections::HashMap, time::Duration};

pub struct HeadController {
    serial_port: Box<dyn SerialPort>,
    io: DynamixelSerialIO,
    io_fan: DynamixelSerialIO,

    xl320_ids: [u8; 2],
    xl320_params: HashMap<u8, MotorModel>,
    xl320_present_temperature: Cache<u8, f64>,
    xl320_pid: Cache<u8, PID>,
    xl320_torque_on: Cache<u8, bool>,
    xl320_goal_position: Cache<u8, f64>,
    xl320_moving_speed: Cache<u8, f64>,
    xl320_torque_limit: Cache<u8, f64>,

    fan_id: u8,
    fan_state: Cache<u8, [bool; 2]>,
}

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

impl HeadController {
    pub fn new(
        serial_port: &str,
        xl320_ids: [u8; 2],
        offsets: [f64; 2],
        is_direct: [bool; 2],
        reductions: [f64; 2],
        fan_id: u8,
    ) -> Result<Self> {
        let serial_port = serialport::new(serial_port, 1_000_000)
            .timeout(Duration::from_millis(10))
            .open()?;
        let io = DynamixelSerialIO::v2();
        let io_fan = DynamixelSerialIO::v1();

        let xl320_params = izip!(xl320_ids, offsets, is_direct, reductions)
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

        Ok(HeadController {
            serial_port,
            io,
            io_fan,
            xl320_ids,
            xl320_params,
            xl320_present_temperature: Cache::with_expiry_duration(Duration::from_secs(1)),
            xl320_pid: Cache::keep_last(),
            xl320_torque_on: Cache::with_expiry_duration(Duration::from_secs(1)),
            xl320_goal_position: Cache::keep_last(),
            xl320_moving_speed: Cache::keep_last(),
            xl320_torque_limit: Cache::keep_last(),

            fan_id,
            fan_state: Cache::keep_last(),
        })
    }
    pub fn get_xl320_present_position_speed_load(
        &mut self,
    ) -> Result<([f64; 2], [f64; 2], [f64; 2])> {
        let (pos, speed, load): (Vec<f64>, Vec<f64>, Vec<f64>) = multiunzip(
            xl320::sync_read_present_position_speed_load(
                &self.io,
                self.serial_port.as_mut(),
                &self.xl320_ids,
            )?
            .iter()
            .enumerate()
            .map(|(i, (p, s, l))| {
                let xl320 = &self.xl320_params[&self.xl320_ids[i]];

                (
                    xl320.to_global_position(xl320::conv::xl320_pos_to_radians(*p)),
                    xl320.to_global_speed(xl320::conv::xl320_oriented_speed_to_rad_per_sec(*s)),
                    xl320.to_global_load(xl320::conv::xl320_load_to_oriented_torque(*l)),
                )
            }),
        );

        Ok((
            pos.try_into().unwrap(),
            speed.try_into().unwrap(),
            load.try_into().unwrap(),
        ))
    }
    pub fn set_xl320_target_position(&mut self, target_position: [f64; 2]) -> Result<()> {
        // Keep only the new goal position
        let current_target_pos = self.get_xl320_target_position()?;
        let target_to_update = differences(&self.xl320_ids, &current_target_pos, &target_position);

        // Keep only the goal position where the motor torque is actually on
        let torques = self.is_xl320_torque_on()?;
        let torques: HashMap<&u8, bool> = self.xl320_ids.iter().zip(torques).collect();
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
                        xl320::conv::radians_to_xl320_pos(
                            self.xl320_params[k].to_local_position(*v),
                        ),
                    )
                })
                .unzip();

            // Send new values to the motors
            xl320::sync_write_goal_position(&self.io, self.serial_port.as_mut(), &keys, &values)?;
            // Update our cache
            for (k, v) in target_to_update {
                self.xl320_goal_position.insert(*k, *v);
            }
        }
        Ok(())
    }
    pub fn set_xl320_moving_speed(&mut self, moving_speed: [f64; 2]) -> Result<()> {
        let current_moving_speed = self.get_xl320_moving_speed()?;
        let moving_speed_to_update =
            differences(&self.xl320_ids, &current_moving_speed, &moving_speed);
        if !moving_speed_to_update.is_empty() {
            let mut err = None;

            for (k, v) in moving_speed_to_update {
                let dxl_v = xl320::conv::rad_per_sec_to_xl320_abs_speed(
                    self.xl320_params[&k].to_local_max_speed(v),
                );

                match xl320::write_moving_speed(&self.io, self.serial_port.as_mut(), k, dxl_v) {
                    Ok(_) => {
                        self.xl320_moving_speed.insert(k, v);
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
    pub fn set_xl320_torque_limit(&mut self, torque_limit: [f64; 2]) -> Result<()> {
        let current_torque_limit = self.get_xl320_torque_limit()?;
        let torque_limit_to_update =
            differences(&self.xl320_ids, &current_torque_limit, &torque_limit);
        if !torque_limit_to_update.is_empty() {
            let mut err = None;

            for (k, v) in torque_limit_to_update {
                let dxl_t = xl320::conv::torque_to_xl320_abs_load(v);

                match xl320::write_torque_limit(&self.io, self.serial_port.as_mut(), k, dxl_t) {
                    Ok(_) => {
                        self.xl320_torque_limit.insert(k, v);
                    },
                    Err(e) => {
                        err = Some(e);
                    },
                }
            }
            if let Some(err) = err {
                return Err(err);
            }
        }

        Ok(())
    }
    pub fn set_xl320_target_position_speed_load(
        &mut self,
        target_position: [f64; 2],
        moving_speed: [f64; 2],
        torque_limit: [f64; 2],
    ) -> Result<()> {
        self.set_xl320_target_position(target_position)?;
        self.set_xl320_moving_speed(moving_speed)?;
        self.set_xl320_torque_limit(torque_limit)?;
        Ok(())
    }
    pub fn get_xl320_temperature(&mut self) -> Result<[f64; 2]> {
        Ok(self
            .xl320_present_temperature
            .entries(&self.xl320_ids)
            .or_try_insert_with(|ids| {
                Ok(
                    xl320::sync_read_present_temperature(&self.io, self.serial_port.as_mut(), ids)?
                        .iter()
                        .map(|&t| t as f64)
                        .collect(),
                )
            })?
            .as_slice()
            .try_into()
            .unwrap())
    }
    pub fn is_xl320_torque_on(&mut self) -> Result<[bool; 2]> {
        Ok(self
            .xl320_torque_on
            .entries(&self.xl320_ids)
            .or_try_insert_with(|ids| {
                Ok(
                    xl320::sync_read_torque_enable(&self.io, self.serial_port.as_mut(), ids)?
                        .iter()
                        .map(|&t| t != 0)
                        .collect(),
                )
            })?
            .as_slice()
            .try_into()
            .unwrap())
    }
    pub fn set_xl320_torque(&mut self, torque: [bool; 2]) -> Result<()> {
        let current_torque = self.is_xl320_torque_on()?;
        let torque_to_update = differences(&self.xl320_ids, &current_torque, &torque);

        if !torque_to_update.is_empty() {
            let mut err = None;

            for (k, v) in torque_to_update {
                match xl320::write_torque_enable(&self.io, self.serial_port.as_mut(), k, v as u8) {
                    Ok(_) => {
                        self.xl320_torque_on.insert(k, v);
                    },
                    Err(e) => {
                        err = Some(e);
                    },
                }
            }

            if let Some(err) = err {
                return Err(err);
            }
        }

        Ok(())
    }
    pub fn get_xl320_pid(&mut self) -> Result<Vec<PID>> {
        self.xl320_pid
            .entries(&self.xl320_ids)
            .or_try_insert_with(|ids| {
                let xl320_p = xl320::sync_read_p_gain(&self.io, self.serial_port.as_mut(), ids)?;
                let xl320_i = xl320::sync_read_i_gain(&self.io, self.serial_port.as_mut(), ids)?;
                let xl320_d = xl320::sync_read_d_gain(&self.io, self.serial_port.as_mut(), ids)?;

                Ok(izip!(xl320_p, xl320_i, xl320_d)
                    .map(|(p, i, d)| PID {
                        p: p as f64,
                        i: i as f64,
                        d: d as f64,
                    })
                    .collect())
            })
    }
    pub fn set_xl320_pid(&mut self, pid: &Vec<PID>) -> Result<()> {
        assert_eq!(pid.len(), 2);

        let current_pid = self.get_xl320_pid()?;
        let pid_to_update = differences(&self.xl320_ids, &current_pid, pid);

        if !pid_to_update.is_empty() {
            let mut err = None;

            for (k, v) in pid_to_update {
                let mut is_ok = true;

                if let Err(e) = xl320::write_p_gain(&self.io, self.serial_port.as_mut(), k, v.p as u8) {
                    err = Some(e);
                    is_ok = false;
                }
                if let Err(e) = xl320::write_i_gain(&self.io, self.serial_port.as_mut(), k, v.i as u8) {
                    err = Some(e);
                    is_ok = false;
                }
                if let Err(e) = xl320::write_d_gain(&self.io, self.serial_port.as_mut(), k, v.d as u8) {
                    err = Some(e);
                    is_ok = false;
                }

                if is_ok {
                    self.xl320_pid.insert(k, v);
                }
            }

            if let Some(err) = err {
                return Err(err);
            }
        }

        Ok(())
    }
    pub fn get_xl320_target_position(&mut self) -> Result<[f64; 2]> {
        Ok(self
            .xl320_goal_position
            .entries(&self.xl320_ids)
            .or_try_insert_with(|ids| {
                Ok(
                    xl320::sync_read_goal_position(&self.io, self.serial_port.as_mut(), ids)?
                        .iter()
                        .enumerate()
                        .map(|(i, &p)| {
                            self.xl320_params[&ids[i]]
                                .to_global_position(xl320::conv::xl320_pos_to_radians(p))
                        })
                        .collect::<Vec<f64>>(),
                )
            })?
            .as_slice()
            .try_into()
            .unwrap())
    }
    pub fn get_xl320_moving_speed(&mut self) -> Result<[f64; 2]> {
        Ok(self
            .xl320_moving_speed
            .entries(&self.xl320_ids)
            .or_try_insert_with(|ids| {
                Ok(
                    xl320::sync_read_moving_speed(&self.io, self.serial_port.as_mut(), ids)?
                        .iter()
                        .enumerate()
                        .map(|(i, &s)| {
                            self.xl320_params[&ids[i]]
                                .to_global_max_speed(xl320::conv::xl320_abs_speed_to_rad_per_sec(s))
                        })
                        .collect::<Vec<f64>>(),
                )
            })?
            .as_slice()
            .try_into()
            .unwrap())
    }
    pub fn get_xl320_torque_limit(&mut self) -> Result<[f64; 2]> {
        Ok(self
            .xl320_torque_limit
            .entries(&self.xl320_ids)
            .or_try_insert_with(|ids| {
                Ok(
                    xl320::sync_read_torque_limit(&self.io, self.serial_port.as_mut(), ids)?
                        .iter()
                        .map(|&l| xl320::conv::xl320_load_to_abs_torque(l))
                        .collect::<Vec<f64>>(),
                )
            })?
            .as_slice()
            .try_into()
            .unwrap())
    }

    pub fn get_fan_state(&mut self) -> Result<[bool; 2]> {
        self.fan_state.entry(self.fan_id).or_try_insert_with(|&id| {
            Ok([
                l0_force_fan::read_fan1_state(&self.io_fan, self.serial_port.as_mut(), id)? != 0,
                l0_force_fan::read_fan2_state(&self.io_fan, self.serial_port.as_mut(), id)? != 0,
            ])
        })
    }
    pub fn set_fan_state(&mut self, state: [bool; 2]) -> Result<()> {
        let current_state = self.get_fan_state()?;
        let mut need_update = false;

        if state[0] != current_state[0] {
            l0_force_fan::write_fan1_state(
                &self.io_fan,
                self.serial_port.as_mut(),
                self.fan_id,
                state[0] as u8,
            )?;
            need_update = true;
        }

        if state[1] != current_state[1] {
            l0_force_fan::write_fan2_state(
                &self.io_fan,
                self.serial_port.as_mut(),
                self.fan_id,
                state[1] as u8,
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
