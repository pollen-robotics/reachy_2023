#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

extern "C" {

uint32_t neck_hwi_init(const char *serial_port, double *zero_disk_offset);

int32_t neck_hwi_is_torque_on(uint32_t uid, double *is_on);

int32_t neck_hwi_set_torque(uint32_t uid, double *torque);

int32_t neck_hwi_get_present_rpy_position(uint32_t uid, double *position);

int32_t neck_hwi_get_target_rpy_position(uint32_t uid, double *target_position);

int32_t neck_hwi_get_present_rpy_velocity(uint32_t uid, double *velocity);

int32_t neck_hwi_get_present_rpy_effort(uint32_t uid, double *effort);

int32_t neck_hwi_set_target_rpy_position(uint32_t uid, double *target_position);

int32_t neck_hwi_get_temperature(uint32_t uid, double *temperature);

int32_t neck_hwi_get_speed_limit(uint32_t uid, double *speed_limit);

int32_t neck_hwi_set_speed_limit(uint32_t uid, double *speed_limit);

int32_t neck_hwi_get_torque_limit(uint32_t uid, double *torque_limit);

int32_t neck_hwi_set_torque_limit(uint32_t uid, double *torque_limit);

int32_t neck_hwi_get_pid(uint32_t uid, double *p, double *i, double *d);

int32_t neck_hwi_set_pid(uint32_t uid, double *p, double *i, double *d);

} // extern "C"
