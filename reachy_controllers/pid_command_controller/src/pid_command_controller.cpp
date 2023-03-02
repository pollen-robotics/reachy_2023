#include "pid_command_controller/pid_command_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"

namespace pid_command_controller
{
using hardware_interface::LoanedCommandInterface;

PIDCommandController::PIDCommandController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{
}

CallbackReturn PIDCommandController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PIDCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::ERROR;
  }


  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PIDCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_)
  {
    command_interfaces_config.names.push_back(joint + "/" + "p_gain");
    command_interfaces_config.names.push_back(joint + "/" + "i_gain");
    command_interfaces_config.names.push_back(joint + "/" + "d_gain");
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
PIDCommandController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

CallbackReturn PIDCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names_, "p_gain", ordered_interfaces) ||
    command_interfaces_.size() != 3 * ordered_interfaces.size())
  {
    RCLCPP_ERROR(
		 get_node()->get_logger(), "Expected %zu position command interfaces, got %zu", joint_names_.size(),
      ordered_interfaces.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PIDCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type PIDCommandController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
			  get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (size_t index = 0; index < command_interfaces_.size(); ++index)
  {
    command_interfaces_[index].set_value((*joint_commands)->data[index]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace pid_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pid_command_controller::PIDCommandController, controller_interface::ControllerInterface)
