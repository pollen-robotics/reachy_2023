#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

extern "C" {

uint32_t arm_hwi_init(const char *serial_port,
                      uint8_t *mx_ids,
                      double *offsets,
                      bool *is_direct,
                      double *reductions,
                      uint8_t fan_id,
                      uint8_t force_sensor_id);

int32_t arm_hwi_get_mx_present_position_speed_load(uint32_t uid,
                                                   double *position,
                                                   double *speed,
                                                   double *load);

int32_t arm_hwi_get_goal_position(uint32_t uid, double *goal_position);

int32_t arm_hwi_get_moving_speed(uint32_t uid, double *moving_speed);

int32_t arm_hwi_get_torque_limit(uint32_t uid, double *torque_limit);

int32_t arm_hwi_set_mx_target_position_speed_load(uint32_t uid,
                                                  double *target_position,
                                                  double *moving_speed,
                                                  double *torque_limit);

int32_t arm_hwi_set_mx_torque_limit(uint32_t uid, double *torque_limit);

int32_t arm_hwi_set_mx_speed_limit(uint32_t uid, double *speed_limit);

int32_t arm_hwi_get_mx_temperature(uint32_t uid, double *temperature);

int32_t arm_hwi_is_mx_torque_on(uint32_t uid, double *is_on);

int32_t arm_hwi_set_mx_torque(uint32_t uid, double *on);

int32_t arm_hwi_get_mx_pid(uint32_t uid, double *p, double *i, double *d);

int32_t arm_hwi_set_mx_pid(uint32_t uid, double *p, double *i, double *d);

int32_t arm_hwi_read_force_sensor(uint32_t uid, double *force);

int32_t arm_hwi_get_fan_state(uint32_t uid, double *fan_state);

int32_t arm_hwi_set_fan_state(uint32_t uid, double *fan_state);

} // extern "C"
