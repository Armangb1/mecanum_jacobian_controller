// Copyright (c) 2023, PAL Robotics
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

#include "mecanum_jacobian/jacobian_controller.hpp"

#include <algorithm>
#include <cmath>

#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mecanum_jacobian
{

  //==================================================OK======================================================
controller_interface::CallbackReturn JacobianController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<jacobian_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
    
    // Initialize Jacobian matrix
    initializeJacobianMatrix();
    
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
    
    RCLCPP_INFO(get_node()->get_logger(), "JacobianController initialized successfully");
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JacobianController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  
  // Validate that we have exactly 4 joints for mecanum wheels
  if (params_.joints.size() != 4)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), 
      "JacobianController requires exactly 4 joints, got %zu", 
      params_.joints.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Set up state interface names for joint velocities
  state_interface_names_.clear();
  for (const std::string& joint_name : params_.joints)
  {
    state_interface_names_.push_back(joint_name + "/velocity");
  }

  RCLCPP_INFO(
    get_node()->get_logger(), 
    "Configured JacobianController with joints: [%s, %s, %s, %s]",
    params_.joints[0].c_str(), params_.joints[1].c_str(), 
    params_.joints[2].c_str(), params_.joints[3].c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JacobianController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize odometry
  x_pos_ = 0.0;
  y_pos_ = 0.0;
  theta_ = 0.0;
  x_velocity_ = 0.0;
  y_velocity_ = 0.0;
  phi_velocity_ = 0.0;
  
  last_update_time_ = get_node()->now();
  
  RCLCPP_INFO(get_node()->get_logger(), "JacobianController activated successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JacobianController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset velocities
  x_velocity_ = 0.0;
  y_velocity_ = 0.0;
  phi_velocity_ = 0.0;
  
  RCLCPP_INFO(get_node()->get_logger(), "JacobianController deactivated successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration
JacobianController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}



controller_interface::InterfaceConfiguration
JacobianController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_names_;
  return state_interfaces_config;
}

bool JacobianController::on_set_chained_mode(bool chained_mode) 
{ 
  is_chained_mode_ = chained_mode;
  
  if (is_chained_mode_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Switching to chained mode");
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Switching to unchained mode");
  }
  
  return true;
}
// ===================================================================================

controller_interface::return_type JacobianController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Get current joint velocities
  std::vector<double> joint_velocities(4);
  for (size_t i = 0; i < state_interfaces_.size() && i < 4; ++i)
  {
    joint_velocities[i] = state_interfaces_[i].get_value();
  }
  
  // Compute body frame velocities using Jacobian
  computeBodyVelocities(joint_velocities);
  
  // Update odometry
  updateOdometry(period);
  
  // Publish TF transform
  publishTransform(time);
  
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
JacobianController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  // No reference interfaces for this controller
  return reference_interfaces;
}

// =======================================================================ok==============================
// NOTE: the states are body velocities
std::vector<hardware_interface::StateInterface>
JacobianController::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Export the computed velocities as state interfaces
  state_interfaces.push_back(
    hardware_interface::StateInterface(
      get_node()->get_name(), "x/velocity", &x_velocity_));
  
  state_interfaces.push_back(
    hardware_interface::StateInterface(
      get_node()->get_name(), "y/velocity", &y_velocity_));
      
  state_interfaces.push_back(
    hardware_interface::StateInterface(
      get_node()->get_name(), "phi/velocity", &phi_velocity_));

  return state_interfaces;
}

controller_interface::return_type JacobianController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // No subscribers for this controller
  return controller_interface::return_type::OK;
}

void JacobianController::initializeJacobianMatrix()
{
  // Initialize the Jacobian matrix based on your Matlab code:
  // Jp = [-1, -1, 1, 1; 1, -1, -1, 1; 1/(b+a), 1/(b+a), 1/(b+a), 1/(b+a)]
  
  // Row 0: X velocity coefficients
  jacobian_matrix_[0][0] = 1.0;
  jacobian_matrix_[0][1] = -1.0;
  jacobian_matrix_[0][2] = -1.0;
  jacobian_matrix_[0][3] = 1.0;
  
  
  // Row 1: Y velocity coefficients  
  jacobian_matrix_[1][0] = 1.0;
  jacobian_matrix_[1][1] = 1.0;
  jacobian_matrix_[1][2] = -1.0;
  jacobian_matrix_[1][3] = -1.0;
  
  // Row 2: Angular velocity coefficients
  double angular_coeff = 1.0 / (HALF_WIDTH + HALF_LENGTH);
  jacobian_matrix_[2][0] = angular_coeff;
  jacobian_matrix_[2][1] = angular_coeff;
  jacobian_matrix_[2][2] = angular_coeff;
  jacobian_matrix_[2][3] = angular_coeff;
  
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Jacobian matrix initialized with a=%.3f, b=%.3f, r=%.3f",
    HALF_LENGTH, HALF_WIDTH, WHEEL_RADIUS);
}

// =============================================================================================================================
void JacobianController::computeBodyVelocities(const std::vector<double>& joint_velocities)
{
  if (joint_velocities.size() != 4)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Expected 4 joint velocities, got %zu", joint_velocities.size());
    return;
  }
  
  // Apply wheel radius and Jacobian transformation
  // Body velocities = Jacobian * (wheel_radius * joint_velocities)
  
  x_velocity_ = 0.0;
  y_velocity_ = 0.0;
  phi_velocity_ = 0.0;
  
  for (size_t j = 0; j < 4; ++j)
  {
    double wheel_velocity = WHEEL_RADIUS * joint_velocities[j];
    x_velocity_ += jacobian_matrix_[0][j] * wheel_velocity;
    y_velocity_ += jacobian_matrix_[1][j] * wheel_velocity;
    phi_velocity_ += jacobian_matrix_[2][j] * wheel_velocity;
  }
}

void JacobianController::updateOdometry(const rclcpp::Duration& period)
{
  double dt = period.seconds();
  // Transform body frame velocities to world frame
  double cos_theta = std::cos(theta_);
  double sin_theta = std::sin(theta_);
  
  // World frame velocities
  double world_x_vel = x_velocity_ * cos_theta - y_velocity_ * sin_theta;
  double world_y_vel = x_velocity_ * sin_theta + y_velocity_ * cos_theta;
  
  // Integrate to get position and orientation
  x_pos_ += world_x_vel * dt;
  y_pos_ += world_y_vel * dt;
  theta_ += phi_velocity_ * dt;
  
  // Normalize theta to [-pi, pi]
  while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
  while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
}

void JacobianController::publishTransform(const rclcpp::Time& time)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  
  transform_msg.header.stamp = time;
  transform_msg.header.frame_id = "odom";
  transform_msg.child_frame_id = "base_link";
  
  // Set translation
  transform_msg.transform.translation.x = x_pos_;
  transform_msg.transform.translation.y = y_pos_;
  transform_msg.transform.translation.z = 0.0;
  
  // Set rotation (convert from euler to quaternion)
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, theta_);
  transform_msg.transform.rotation.x = quaternion.x();
  transform_msg.transform.rotation.y = quaternion.y();
  transform_msg.transform.rotation.z = quaternion.z();
  transform_msg.transform.rotation.w = quaternion.w();
  
  // Publish transform
  tf_broadcaster_->sendTransform(transform_msg);
}

}  // namespace mecanum_jacobian

PLUGINLIB_EXPORT_CLASS(
  mecanum_jacobian::JacobianController, controller_interface::ChainableControllerInterface)