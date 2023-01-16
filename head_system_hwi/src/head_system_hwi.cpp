#include "head_system_hwi/head_system_hwi.hpp"

#include "head_hwi.h"

#include <cmath>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

std::vector<float> parse_string_as_vec(std::string s) {
  std::string delimiter = ",";

  std::vector<float> v;

  size_t pos = 0;
  std::string token;
  while ((pos = s.find(delimiter)) != std::string::npos) {
    token = s.substr(0, pos);
    v.push_back(std::stof(token));
    s.erase(0, pos + delimiter.length());
  }
  v.push_back(std::stof(s));

  return v;
}


namespace head_system_hwi
{
CallbackReturn
HeadSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  long unsigned int nb_joints_expected = 2 + 2;
  if (info.joints.size() != nb_joints_expected)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("HeadSystem"),
      "Incorrect number of joints, expected %ld, got \"%s\"",nb_joints_expected,
       std::to_string(info.joints.size()).c_str()
    );
    return CallbackReturn::ERROR;
  }


  const char *serial_port;
  uint8_t mx_ids[2];
  double offsets[2];
  bool is_direct[2];
  double reductions[2];

  uint8_t fan_id;

  for (auto const& params : info.hardware_parameters)
  {
    if (params.first == "serial_port") {
      serial_port = params.second.c_str();
    }
    else if (params.first == "mx_ids") {
      std::vector<float> v = parse_string_as_vec(params.second.c_str());
      for (uint i=0; i < v.size(); i++) {
        mx_ids[i] = (int)v[i];
      }
    }
    else if (params.first == "offsets") {
      std::vector<float> v = parse_string_as_vec(params.second.c_str());
      for (uint i=0; i < v.size(); i++) {
        offsets[i] = v[i];
      }
    }
    else if (params.first == "is_direct") {
      std::vector<float> v = parse_string_as_vec(params.second.c_str());
      for (uint i=0; i < v.size(); i++) {
        is_direct[i] = v[i] != 0.0;
      }
    }

     else if (params.first == "reductions") {
      std::vector<float> v = parse_string_as_vec(params.second.c_str());
      for (uint i=0; i < v.size(); i++) {
        reductions[i] = v[i];
      }
    }
    else if (params.first == "fan_id") {
      fan_id = std::stoi(params.second.c_str());
    }
  }


  RCLCPP_INFO(
    rclcpp::get_logger("HeadSystem"),
    "Trying to connect on serial port \"%s\"",
    serial_port
  );

  this->uid = head_hwi_init(
    serial_port,
    mx_ids, offsets, is_direct, reductions,
    fan_id
  );

  clock_ = rclcpp::Clock();
  RCLCPP_INFO(
    rclcpp::get_logger("HeadSystem"),
    "System \"%s\" init!", info_.name.c_str()
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn
HeadSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set some default values
  for (int i=0; i < 2; i++) {
    hw_mx_states_position_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_velocity_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_effort_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_temperature_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_torque_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_p_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_i_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_d_gain_[i] = std::numeric_limits<double>::quiet_NaN();

  }
  head_hwi_get_goal_position(this->uid, hw_mx_commands_position_);
  head_hwi_get_moving_speed(this->uid, hw_mx_commands_max_speed_);
  head_hwi_get_torque_limit(this->uid, hw_mx_commands_torque_limit_);
  head_hwi_is_mx_torque_on(this->uid, hw_mx_commands_torque_);
  head_hwi_get_mx_pid(this->uid, hw_mx_commands_p_gain_, hw_mx_commands_i_gain_, hw_mx_commands_d_gain_);

  for (int i=0; i < 2; i++) {
    hw_fans_states_[i] = std::numeric_limits<double>::quiet_NaN();
  }
  head_hwi_get_fan_state(this->uid, hw_fans_commands_);

  last_timestamp_ = clock_.now();

  RCLCPP_INFO(
    rclcpp::get_logger("HeadSystem"),
    "System \"%s\" successfully started!", info_.name.c_str()
  );
  return CallbackReturn::SUCCESS;
}

CallbackReturn
HeadSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("HeadSystem"),
    "System \"%s\" successfully deactivated!", info_.name.c_str()
  );
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HeadSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

// MX
  for (std::size_t i = 0; i < 2; i++)
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
      rclcpp::get_logger("HeadSystem"),
      "export state interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

//  // FANS
  for (std::size_t i = 0; i < 2; i++)
  {
    auto joint = info_.joints[2 + i];

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "state", &hw_fans_states_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("HeadSystem"),
      "export state interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
HeadSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

// MX
  for (std::size_t i = 0; i < 2; i++)
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
      rclcpp::get_logger("HeadSystem"),
      "export command interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

//  // FANS
  for (std::size_t i = 0; i < 2; i++)
  {
    auto joint = info_.joints[2 + i];

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, "state", &hw_fans_commands_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("HeadSystem"),
      "export command interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  return command_interfaces;
}

hardware_interface::return_type
HeadSystem::read()
{
  current_timestamp = clock_.now();
  rclcpp::Duration duration = current_timestamp - last_timestamp_;
  last_timestamp_ = current_timestamp;

  if (head_hwi_get_mx_present_position_speed_load(
    this->uid,
    hw_mx_states_position_,
    hw_mx_states_velocity_,
    hw_mx_states_effort_) != 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("HeadSystem"),
        "(%s) READ POS/VEL/EFF ERROR!", info_.name.c_str()
      );
  }


  if (head_hwi_get_mx_temperature(this->uid, hw_mx_states_temperature_)) {
      RCLCPP_INFO(
        rclcpp::get_logger("HeadSystem"),
        "(%s) READ TEMPERATURE ERROR!", info_.name.c_str()
      );
  }

  if (head_hwi_get_mx_pid(
    this->uid,
    hw_mx_states_p_gain_,
    hw_mx_states_i_gain_,
    hw_mx_states_d_gain_)) {
      RCLCPP_INFO(
        rclcpp::get_logger("HeadSystem"),
        "(%s) READ PID ERROR!", info_.name.c_str()
      );
    }

    if (head_hwi_is_mx_torque_on(this->uid, hw_mx_states_torque_)) {
      RCLCPP_INFO(
        rclcpp::get_logger("HeadSystem"),
        "(%s) READ TORQUE ERROR!", info_.name.c_str()
      );
    }


    if (head_hwi_get_fan_state(this->uid, hw_fans_states_) != 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("HeadSystem"),
        "(%s) READ FAN ERROR!", info_.name.c_str()
      );
    }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HeadSystem::write()
{

  if (head_hwi_set_mx_torque(this->uid, hw_mx_commands_torque_) != 0) {
        RCLCPP_INFO(
        rclcpp::get_logger("HeadSystem"),
        "(%s) WRITE TORQUE ERROR!", info_.name.c_str()
      );
  }

  if (head_hwi_set_mx_target_position_speed_load(
    this->uid,
    hw_mx_commands_position_,
    hw_mx_commands_max_speed_,
    hw_mx_commands_torque_limit_) != 0) {
    RCLCPP_INFO(
        rclcpp::get_logger("HeadSystem"),
        "(%s) WRITE POS/SPEED/TORQUE ERROR!", info_.name.c_str()
      );
  }

  if (head_hwi_set_mx_pid(
    this->uid,
    hw_mx_commands_p_gain_,
    hw_mx_commands_i_gain_,
    hw_mx_commands_d_gain_
  ) != 0) {
        RCLCPP_INFO(
        rclcpp::get_logger("HeadSystem"),
        "(%s) WRITE PID ERROR!", info_.name.c_str()
      );
  }

  if (head_hwi_set_fan_state(
    this->uid,
    hw_fans_commands_
  ) != 0) {
    RCLCPP_INFO(
        rclcpp::get_logger("HeadSystem"),
        "(%s) WRITE FAN ERROR!", info_.name.c_str()
      );
  }

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  head_system_hwi::HeadSystem,
  hardware_interface::SystemInterface)
