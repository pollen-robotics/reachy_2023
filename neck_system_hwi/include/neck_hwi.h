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

int32_t neck_hwi_set_target_rpy_position(uint32_t uid, double *target_position);

int32_t neck_hwi_get_temperature(uint32_t uid, double *temperature);

} // extern "C"
