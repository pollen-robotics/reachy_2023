#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

extern "C" {

uint32_t head_hwi_init(const char *serial_port,
                       uint8_t *xl320_ids,
                       double *offsets,
                       bool *is_direct,
                       double *reductions,
                       uint8_t fan_id);

int32_t head_hwi_get_xl320_present_position_speed_load(uint32_t uid,
                                                       double *position,
                                                       double *speed,
                                                       double *load);

int32_t head_hwi_get_goal_position(uint32_t uid, double *goal_position);

int32_t head_hwi_get_moving_speed(uint32_t uid, double *moving_speed);

int32_t head_hwi_get_torque_limit(uint32_t uid, double *torque_limit);

int32_t head_hwi_set_xl320_target_position_speed_load(uint32_t uid,
                                                      double *target_position,
                                                      double *moving_speed,
                                                      double *torque_limit);

int32_t head_hwi_set_xl320_torque_limit(uint32_t uid, double *torque_limit);

int32_t head_hwi_set_xl320_speed_limit(uint32_t uid, double *speed_limit);

int32_t head_hwi_get_xl320_temperature(uint32_t uid, double *temperature);

int32_t head_hwi_is_xl320_torque_on(uint32_t uid, double *is_on);

int32_t head_hwi_set_xl320_torque(uint32_t uid, double *on);

int32_t head_hwi_get_xl320_pid(uint32_t uid, double *p, double *i, double *d);

int32_t head_hwi_set_xl320_pid(uint32_t uid, double *p, double *i, double *d);

int32_t head_hwi_get_fan_state(uint32_t uid, double *fan_state);

int32_t head_hwi_set_fan_state(uint32_t uid, double *fan_state);

} // extern "C"
