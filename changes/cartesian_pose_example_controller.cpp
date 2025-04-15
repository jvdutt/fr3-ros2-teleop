// Copyright (c) 2023 Franka Robotics GmbH
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

#include <franka_example_controllers/cartesian_pose_example_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  // add the robot time interface
  config.names.push_back(arm_id_ + "/robot_time");
  return config;
}

controller_interface::return_type CartesianPoseExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    // Get initial orientation and translation
    std::tie(orientation_, position_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    initial_robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = 0.0;

    orientation_raw = orientation_;
    position_raw = position_;

    orientation_filter1 = orientation_;
    position_filter1 = position_;

    orientation_cmd = orientation_;
    position_cmd = position_;

    initialization_flag_ = false;
  } else {
    robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = robot_time_ - initial_robot_time_;
  }

  position_filter1 = alpha1 * position_filter1 + (1 - alpha1) * position_raw;
  position_cmd = beta * position_cmd + (1 - beta) * position_filter1;

  orientation_filter1 = orientation_filter1.slerp(1-alpha1, orientation_raw);
  orientation_cmd = orientation_cmd.slerp(1-beta, orientation_filter1);

  if (franka_cartesian_pose_->setCommand(orientation_cmd, position_cmd)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

void CartesianPoseExampleController::equilibriumPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
// std::lock_guard<std::mutex> position_d_target_mutex_lock(
//     position_and_orientation_d_target_mutex_);
position_raw << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
Eigen::Quaterniond last_orientation_d_target(orientation_raw);
orientation_raw.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w;
if (last_orientation_d_target.coeffs().dot(orientation_raw.coeffs()) < 0.0) {
  orientation_raw.coeffs() << -orientation_raw.coeffs();
}
}



CallbackReturn CartesianPoseExampleController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  // Equilibrium pose subscription
  sub_equilibrium_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "equilibrium_pose", 20,
    std::bind(&CartesianPoseExampleController::equilibriumPoseCallback, this,
              std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(robot_utils::time_out);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerInterface)

