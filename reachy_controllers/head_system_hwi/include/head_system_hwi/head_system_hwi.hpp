#ifndef _HEAD_SYSTEM_HWI
#define _HEAD_SYSTEM_HWI

#include "rclcpp/macros.hpp"

#include "rclcpp/clock.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#define LOG_THROTTLE_DURATION 30000

namespace head_system_hwi
{
  using namespace hardware_interface;
class HeadSystem : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(HeadSystem)

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
// XL320
  double hw_xl320_states_position_[2];
  double hw_xl320_states_velocity_[2];
  double hw_xl320_states_effort_[2];
  double hw_xl320_states_temperature_[2];
  double hw_xl320_states_torque_limit_[2];
  double hw_xl320_states_speed_limit_[2];
  double hw_xl320_states_torque_[2];
  double hw_xl320_states_p_gain_[2];
  double hw_xl320_states_i_gain_[2];
  double hw_xl320_states_d_gain_[2];

  double hw_xl320_commands_position_[2];
  double hw_xl320_commands_speed_limit_[2];
  double hw_xl320_commands_torque_limit_[2];
  double hw_xl320_commands_torque_[2];
  double hw_xl320_commands_p_gain_[2];
  double hw_xl320_commands_i_gain_[2];
  double hw_xl320_commands_d_gain_[2];

// Fans
  double hw_fans_states_[2];
  double hw_fans_commands_[2];

  // Store time between update loops
  rclcpp::Clock clock_;
  rclcpp::Time last_timestamp_;
  rclcpp::Time current_timestamp;  // Avoid initialization on each read

  uint8_t uid;
};

}

#endif // _HEAD_SYSTEM_HWI
