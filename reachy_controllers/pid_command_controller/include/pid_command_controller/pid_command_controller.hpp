// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PID_COMMAND_CONTROLLER__PID_COMMAND_CONTROLLER_HPP_
#define PID_COMMAND_CONTROLLER__PID_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pid_command_controller/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace pid_command_controller
{
using CmdType = std_msgs::msg::Float64MultiArray;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PIDCommandController : public controller_interface::ControllerInterface
{
public:
  PID_COMMAND_CONTROLLER_PUBLIC
  PIDCommandController();

  PID_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  PID_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  PID_COMMAND_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  PID_COMMAND_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  PID_COMMAND_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  PID_COMMAND_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  PID_COMMAND_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  std::string logger_name_;
};

}  // namespace pid_command_controller

#endif  // PID_COMMAND_CONTROLLER__PID_COMMAND_CONTROLLER_HPP_
