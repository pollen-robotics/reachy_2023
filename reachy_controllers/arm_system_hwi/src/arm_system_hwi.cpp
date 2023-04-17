#include "arm_system_hwi/arm_system_hwi.hpp"

#include "arm_hwi.h"

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


namespace arm_system_hwi
{
int log_throttle_duration = LOG_THROTTLE_DURATION_DEFAULT;
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
  uint8_t mx_ids[8];
  double offsets[8];
  bool is_direct[8];
  double reductions[8];
  uint8_t fan_id;
  uint8_t force_sensor_id;
  
  // log throttle configuration
  const char* env_log_throttle = std::getenv("REACHY_CONTROLLER_LOG_THROTTLE");
  if (env_log_throttle == NULL){
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "REACHY_CONTROLLER_LOG_THROTTLE not set, let's use default value");
  }
  else{
    try{
      log_throttle_duration = std::stoi(env_log_throttle);
	RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "LOG_THROTTLE_DURATION overrode by environment variable");
    }
    catch (const std::invalid_argument & e) {
      std::cout << e.what() << "\n";
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Environment variable could not be converted to int, back to default value");      

    }
    catch (const std::out_of_range & e) {
      std::cout << e.what() << "\n";
              RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Environment variable could not be converted to int, back to default value");
    }
  }
  RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Log throttle duration :: %d", log_throttle_duration);

  
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
    hw_mx_states_torque_limit_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_speed_limit_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_p_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_i_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_mx_states_d_gain_[i] = std::numeric_limits<double>::quiet_NaN();
  }

  // TODO: Check error and retry if any!
  arm_hwi_get_goal_position(this->uid, hw_mx_commands_position_);
  // Dynamixel moving speed here actually means speed limit
  arm_hwi_get_moving_speed(this->uid, hw_mx_commands_speed_limit_);
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
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "torque_limit", &hw_mx_states_torque_limit_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "speed_limit", &hw_mx_states_speed_limit_[i]));       
        

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
        joint.name, "speed_limit", &hw_mx_commands_speed_limit_[i]));
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
ArmSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  current_timestamp = clock_.now();
  rclcpp::Duration duration = current_timestamp - last_timestamp_;
  last_timestamp_ = current_timestamp;

  auto t0 = clock_.now();

  if (arm_hwi_get_mx_present_position_speed_load(
    this->uid,
    hw_mx_states_position_,
    hw_mx_states_velocity_,
    hw_mx_states_effort_) != 0) {
        RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) READ POS/VEL/EFF ERROR!", info_.name.c_str()
      );
  }

  if (arm_hwi_get_mx_temperature(this->uid, hw_mx_states_temperature_)) {
        RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) READ TEMPERATURE ERROR!", info_.name.c_str()
      );
  }
  
  if (arm_hwi_get_torque_limit(this->uid, hw_mx_states_torque_limit_)) {

        RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) READ TORQUE_LIMIT ERROR!", info_.name.c_str()
      );
  }


  if (arm_hwi_get_moving_speed(this->uid, hw_mx_states_speed_limit_)) {
  RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) READ SPEED_LIMIT ERROR!", info_.name.c_str()
      );
  }

  if (arm_hwi_get_mx_pid(
    this->uid, 
    hw_mx_states_p_gain_, 
    hw_mx_states_i_gain_, 
    hw_mx_states_d_gain_)) {
        RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) READ PID ERROR!", info_.name.c_str()
      );
    }

    if (arm_hwi_is_mx_torque_on(this->uid, hw_mx_states_torque_)) {
        RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) READ TORQUE ERROR!", info_.name.c_str()
      );
    }

    if (arm_hwi_read_force_sensor(this->uid, &hw_force_sensor_) != 0) {
        RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) READ FORCE SENSOR ERROR!", info_.name.c_str()
      );
    }

    if (arm_hwi_get_fan_state(this->uid, hw_fans_states_) != 0) {
        RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) READ FAN ERROR!", info_.name.c_str()
      );
    }

  auto t1 = clock_.now();
  rclcpp::Duration dt = t1 - t0;
  RCLCPP_DEBUG(
    rclcpp::get_logger("ArmSystem"),
    "(%s) READ ITER DT %fms", info_.name.c_str(), dt.seconds() * 1000.0    
  );

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  auto t0 = clock_.now();

  if (arm_hwi_set_mx_torque(this->uid, hw_mx_commands_torque_) != 0) {
          RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) WRITE TORQUE ERROR!", info_.name.c_str()
      );
  }

  if (arm_hwi_set_mx_target_position_speed_load(
    this->uid,
    hw_mx_commands_position_,
    hw_mx_commands_speed_limit_,
    hw_mx_commands_torque_limit_) != 0) {
      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) WRITE POS/SPEED/TORQUE ERROR!", info_.name.c_str()
      );
  }

  if (arm_hwi_set_mx_pid(
    this->uid,
    hw_mx_commands_p_gain_,
    hw_mx_commands_i_gain_,
    hw_mx_commands_d_gain_
  ) != 0) {
          RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) WRITE PID ERROR!", info_.name.c_str()
      );
  }

  if (arm_hwi_set_fan_state(
    this->uid,
    hw_fans_commands_
  ) != 0) {
      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
        "(%s) WRITE FAN ERROR!", info_.name.c_str()
      );
  }

  auto t1 = clock_.now();
  rclcpp::Duration dt = t1 - t0;
      RCLCPP_DEBUG_THROTTLE(
        rclcpp::get_logger("ArmSystem"),
        clock_,
        arm_system_hwi::log_throttle_duration,
    "(%s) WRITE ITER DT %fms", info_.name.c_str(), dt.seconds() * 1000.0    
  );

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_system_hwi::ArmSystem,
  hardware_interface::SystemInterface)
