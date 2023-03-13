#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

extern "C" {

uint32_t neck_hwi_init(const char *serial_port,
                       uint8_t id,
                       double alpha,
                       double *hardware_zero,
                       double reduction);

int32_t neck_hwi_get_orientation(uint32_t uid, double *orientation);

int32_t neck_hwi_get_goal_orientation(uint32_t uid, double *target_orientation);

int32_t neck_hwi_set_target_orientation_max_speed(uint32_t uid,
                                                  double *target_orientation,
                                                  double *speed_limit);

int32_t neck_hwi_get_max_speed(uint32_t uid, double *max_speed);

int32_t neck_hwi_is_torque_on(uint32_t uid, double *is_on);

int32_t neck_hwi_set_torque(uint32_t uid, double *on);

int32_t neck_hwi_get_pid(uint32_t uid, double *p, double *i, double *d);

int32_t neck_hwi_set_pid(uint32_t uid, double *p, double *i, double *d);

} // extern "C"
