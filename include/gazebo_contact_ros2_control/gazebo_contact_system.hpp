// Copyright (c) 2024, The ICube Laboratory, Strasbourg University. All rights
// reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Open Source Robotics Foundation nor the names of
//    its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
   Author: Manuel YGUEL
   Desc:   Extension of Gazebo plugin for ros_control that allows
   'hardware_interfaces' to be plugged in using pluginlib, integrating contact
   sensors.
*/

#ifndef GAZEBO_ROS2_CONTROL_CONTACT__GAZEBO_CONTACT_SYSTEM_HPP_
#define GAZEBO_ROS2_CONTROL_CONTACT__GAZEBO_CONTACT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"

#include "gazebo_ros2_control/gazebo_system_interface.hpp"

#include "std_msgs/msg/bool.hpp"

namespace gazebo_ros2_control
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Forward declaration
class GazeboContactSystemPrivate;

// These class must inherit `gazebo_ros2_control::GazeboSystemInterface` which
// implements a simulated `ros2_control` `hardware_interface::SystemInterface`.

class GazeboContactSystem : public GazeboSystemInterface
{
 public:
  // Documentation Inherited
  CallbackReturn
  on_init(const hardware_interface::HardwareInfo &system_info) override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // Documentation Inherited
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  // Documentation Inherited
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // Documentation Inherited
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) override;

  // Documentation Inherited
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  // Documentation Inherited
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  // Documentation Inherited
  bool initSim(rclcpp::Node::SharedPtr &model_nh,
               gazebo::physics::ModelPtr parent_model,
               const hardware_interface::HardwareInfo &hardware_info,
               sdf::ElementPtr sdf) override;

 private:
  void registerJoints(const hardware_interface::HardwareInfo &hardware_info,
                      gazebo::physics::ModelPtr parent_model);

  void registerSensors(const hardware_interface::HardwareInfo &hardware_info,
                       gazebo::physics::ModelPtr parent_model);

  /// \brief Private data class
  std::unique_ptr<GazeboContactSystemPrivate> dataPtr;
};

} // namespace gazebo_ros2_control

#endif // GAZEBO_ROS2_CONTROL_CONTACT__GAZEBO_CONTACT_SYSTEM_HPP_