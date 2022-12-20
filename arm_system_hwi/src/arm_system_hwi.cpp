#include "arm_system_hwi/arm_system_hwi.hpp"

#include "arm_hwi.h"

#include <cmath>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"



namespace arm_system_hwi
{
CallbackReturn
ArmSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  if (info.joints.size() != 8 + 3)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArmSystem"),
      "Exactly 11 joints should be provided (8 motors and 3 fans)!"
    );
    return CallbackReturn::ERROR;
  }

  if (info.sensors.size() != 1)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArmSystem"),
      "Exactly 1 sensor should be provided (force_sensor)!"
    );
    return CallbackReturn::ERROR;
  }

  const char *serial_port;
  uint8_t mx_ids[8] = {10, 11, 12, 13, 14, 15, 16, 17};
  double offsets[8] = {1.57, 1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool is_direct[8] = {false, false, false, false, false, false, false, true};
  double reductions[8] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -2.0};
  uint8_t fan_id;
  uint8_t force_sensor_id;

  for (auto const& params : info.hardware_parameters)
  {
    if (params.first == "serial_port") {
      serial_port = params.second.c_str();
    }
    else if (params.first == "mx_ids") {

    }
    else if (params.first == "offsets") {

    }
    else if (params.first == "is_direct") {

    }
    else if (params.first == "reductions") {

    }
    else if (params.first == "fan_id") {
      fan_id = std::stoi(params.second.c_str());
    }
    else if (params.first == "force_sensor_id") {
      force_sensor_id = std::stoi(params.second.c_str());
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ArmSystem"),
    "Trying to connect on serial port \"%s\"",
    serial_port
  );

  this->uid = arm_hwi_init(
    serial_port,
    mx_ids, offsets, is_direct, reductions, 
    fan_id, 
    force_sensor_id
  );

  clock_ = rclcpp::Clock();
  RCLCPP_INFO(
    rclcpp::get_logger("ArmSystem"),
    "System \"%s\" init!", info_.name.c_str()
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn
ArmSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set some default values
  for (int i=0; i < 8; i++) {
    hw_mx_states_position_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_velocity_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_effort_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_temperature_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_torque_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_p_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_i_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_d_gain_[i] = std::numeric_limits<double>::quiet_NaN();
  }
  arm_hwi_get_goal_position(this->uid, hw_mx_commands_position_);
  arm_hwi_get_moving_speed(this->uid, hw_mx_commands_max_speed_);
  arm_hwi_get_torque_limit(this->uid, hw_mx_commands_torque_limit_);
  arm_hwi_is_mx_torque_on(this->uid, hw_mx_commands_torque_);
  arm_hwi_get_mx_pid(this->uid, hw_mx_commands_p_gain_, hw_mx_commands_i_gain_, hw_mx_commands_d_gain_);
  
  hw_force_sensor_ = std::numeric_limits<double>::quiet_NaN();

  for (int i=0; i < 3; i++) {
    hw_fans_states_[i] = std::numeric_limits<double>::quiet_NaN();
  }
  arm_hwi_get_fan_state(this->uid, hw_fans_commands_);

  last_timestamp_ = clock_.now();

  RCLCPP_INFO(
    rclcpp::get_logger("ArmSystem"),
    "System \"%s\" successfully started!", info_.name.c_str()
  );
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ArmSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ArmSystem"),
    "System \"%s\" successfully deactivated!", info_.name.c_str()
  );
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

// MX
  for (std::size_t i = 0; i < 8; i++)
  {
    auto joint = info_.joints[i];

      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &hw_mx_states_position_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &hw_mx_states_velocity_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &hw_mx_states_effort_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "temperature", &hw_mx_states_temperature_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "torque", &hw_mx_states_torque_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "p_gain", &hw_mx_states_p_gain_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "i_gain", &hw_mx_states_i_gain_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "d_gain", &hw_mx_states_d_gain_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("ArmSystem"),
      "export state interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  // FORCE SENSOR
  auto force_sensor = info_.sensors[0];
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    force_sensor.name, "force", &hw_force_sensor_));

    RCLCPP_INFO(
      rclcpp::get_logger("ArmSystem"),
      "export state interface (%s) \"%s\"!", info_.name.c_str(), force_sensor.name.c_str()
      );

  // FANS 
  for (std::size_t i = 0; i < 3; i++)
  {
    auto joint = info_.joints[8 + i];

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "state", &hw_fans_states_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("ArmSystem"),
      "export state interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

// MX
  for (std::size_t i = 0; i < 8; i++)
  {
    auto joint = info_.joints[i];

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &hw_mx_commands_position_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, "torque", &hw_mx_commands_torque_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, "max_speed", &hw_mx_commands_max_speed_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, "torque_limit", &hw_mx_commands_torque_limit_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, "p_gain", &hw_mx_commands_p_gain_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, "i_gain", &hw_mx_commands_i_gain_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, "d_gain", &hw_mx_commands_d_gain_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("ArmSystem"),
      "export command interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  // FANS 
  for (std::size_t i = 0; i < 3; i++)
  {
    auto joint = info_.joints[8 + i];

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, "state", &hw_fans_commands_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("ArmSystem"),
      "export command interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  return command_interfaces;
}

hardware_interface::return_type
ArmSystem::read()
{
  current_timestamp = clock_.now();
  rclcpp::Duration duration = current_timestamp - last_timestamp_;
  last_timestamp_ = current_timestamp;

  if (arm_hwi_get_mx_present_position_speed_load(
    this->uid,
    hw_mx_states_position_,
    hw_mx_states_velocity_,
    hw_mx_states_effort_) != 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("ArmSystem"),
        "(%s) READ POS/VEL/EFF ERROR!", info_.name.c_str()
      );
  }

  if (arm_hwi_get_mx_temperature(this->uid, hw_mx_states_temperature_)) {
      RCLCPP_INFO(
        rclcpp::get_logger("ArmSystem"),
        "(%s) READ TEMPERATURE ERROR!", info_.name.c_str()
      );
  }

  if (arm_hwi_get_mx_pid(
    this->uid, 
    hw_mx_states_p_gain_, 
    hw_mx_states_i_gain_, 
    hw_mx_states_d_gain_)) {
      RCLCPP_INFO(
        rclcpp::get_logger("ArmSystem"),
        "(%s) READ PID ERROR!", info_.name.c_str()
      );
    }

    if (arm_hwi_is_mx_torque_on(this->uid, hw_mx_states_torque_)) {
      RCLCPP_INFO(
        rclcpp::get_logger("ArmSystem"),
        "(%s) READ TORQUE ERROR!", info_.name.c_str()
      );
    }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ArmSystem::write()
{
  if (arm_hwi_set_mx_target_position_speed_load(
    this->uid,
    hw_mx_commands_position_,
    hw_mx_commands_max_speed_,
    hw_mx_commands_torque_limit_) != 0) {
    RCLCPP_INFO(
        rclcpp::get_logger("ArmSystem"),
        "(%s) WRITE POS/SPEED/TORQUE ERROR!", info_.name.c_str()
      );
  }

  // double hw_mx_commands_position_[8];
  // double hw_mx_commands_torque_[8];
  // double hw_mx_commands_p_gain_[8];
  // double hw_mx_commands_i_gain_[8];
  // double hw_mx_commands_d_gain_[8];


//   if (!std::isnan(hw_commands_position_grasp_)) {
//     if (gripper_write_grasping_finger_position(this->id, hw_commands_position_grasp_) != 0) {
//       RCLCPP_INFO(
//         rclcpp::get_logger("ArmSystem"),
//         "(%s) WRITE GRASP POS ERROR!", info_.name.c_str()
//       );
//     }
//   }
//   if (!std::isnan(hw_commands_position_action_)) {
//     if (gripper_write_action_finger_position(this->id, hw_commands_position_action_) != 0) {
//       RCLCPP_INFO(
//         rclcpp::get_logger("ArmSystem"),
//         "(%s) WRITE ACTION POS ERROR!", info_.name.c_str()
//       );
//     }
//   }

//   bool torque = true ? hw_commands_torque_ == 1.0 : false;

//   if (gripper_set_torque(this->id, torque) != 0) {
//     RCLCPP_INFO(
//       rclcpp::get_logger("ArmSystem"),
//       "(%s) SET TORQUE ERROR!", info_.name.c_str()
//     );
//   }

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_system_hwi::ArmSystem,
  hardware_interface::SystemInterface)