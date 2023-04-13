#ifndef _ARM_SYSTEM_HWI
#define _ARM_SYSTEM_HWI

#include "rclcpp/macros.hpp"

#include "rclcpp/clock.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#define LOG_THROTTLE_DURATION_DEFAULT 30000

namespace arm_system_hwi
{

using namespace hardware_interface;
  
class ArmSystem : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystem)

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
// MX
  double hw_mx_states_position_[8];
  double hw_mx_states_velocity_[8];
  double hw_mx_states_effort_[8];
  double hw_mx_states_temperature_[8];
  double hw_mx_states_speed_limit_[8];
  double hw_mx_states_torque_limit_[8];
  double hw_mx_states_torque_[8];
  double hw_mx_states_p_gain_[8];
  double hw_mx_states_i_gain_[8];
  double hw_mx_states_d_gain_[8];

  double hw_mx_commands_position_[8];
  double hw_mx_commands_speed_limit_[8];
  double hw_mx_commands_torque_limit_[8];
  double hw_mx_commands_torque_[8];
  double hw_mx_commands_p_gain_[8];
  double hw_mx_commands_i_gain_[8];
  double hw_mx_commands_d_gain_[8];

// Force Sensor
  double hw_force_sensor_;

// Fans
  double hw_fans_states_[3];
  double hw_fans_commands_[3];

  // Store time between update loops
  rclcpp::Clock clock_;
  rclcpp::Time last_timestamp_;
  rclcpp::Time current_timestamp;  // Avoid initialization on each read

  uint8_t uid;
};

}

#endif // _ARM_SYSTEM_HWI
