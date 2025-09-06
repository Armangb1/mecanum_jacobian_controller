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

#include "mecanum_jacobian/inverse_jacobian_controller.hpp"

#include <algorithm>
#include <cmath>

#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mecanum_jacobian
{
controller_interface::CallbackReturn InverseJacobianController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<inverse_jacobian_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
    
    // Initialize inverse Jacobian matrix
    initializeInverseJacobianMatrix();
    
    // Initialize wheel velocities vector
    wheel_velocities_.resize(4, 0.0);
    
    RCLCPP_INFO(get_node()->get_logger(), "InverseJacobianController initialized successfully");
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn InverseJacobianController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  
  // Validate that we have exactly 4 joints for mecanum wheels
  if (params_.joints.size() != 4)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), 
      "InverseJacobianController requires exactly 4 joints, got %zu", 
      params_.joints.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Set up command interface names for joint velocities
  command_interface_names_.clear();
  for (const std::string& joint_name : params_.joints)
  {
    command_interface_names_.push_back(joint_name + "/voltage");
  }

  // Create velocity command subscriber
  velocity_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    params_.velocity_command_topic, 
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
      velocityCommandCallback(msg);
    });

  // Initialize realtime buffer
  velocity_command_ptr_.writeFromNonRT(std::shared_ptr<geometry_msgs::msg::Twist>());

  RCLCPP_INFO(
    get_node()->get_logger(), 
    "Configured InverseJacobianController with joints: [%s, %s, %s, %s], topic: %s",
    params_.joints[0].c_str(), params_.joints[1].c_str(), 
    params_.joints[2].c_str(), params_.joints[3].c_str(),
    params_.velocity_command_topic.c_str());

    reference_interface_names_ = {"ref_x/velocity", "ref_y/velocity", "ref_phi/velocity"};
    reference_interfaces_.clear();
    for (const std::string& ref_name : reference_interface_names_)
    {
      reference_interfaces_.emplace_back(
        hardware_interface::CommandInterface(
          get_node()->get_name(), ref_name, &ref_x_velocity_));
    }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InverseJacobianController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset all velocities
  std::fill(wheel_velocities_.begin(), wheel_velocities_.end(), 0.0);
  ref_x_velocity_ = 0.0;
  ref_y_velocity_ = 0.0;
  ref_phi_velocity_ = 0.0;
  
  RCLCPP_INFO(get_node()->get_logger(), "InverseJacobianController activated successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InverseJacobianController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set all command interfaces to zero
  for (auto & command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "InverseJacobianController deactivated successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
InverseJacobianController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
InverseJacobianController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return state_interfaces_config;
}

bool InverseJacobianController::on_set_chained_mode(bool chained_mode) 
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

controller_interface::return_type InverseJacobianController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Compute wheel velocities from body frame velocities
  computeWheelVelocities();
  
  // Set command interfaces
  for (size_t i = 0; i < command_interfaces_.size() && i < wheel_velocities_.size(); ++i)
  {
    command_interfaces_[i].set_value(wheel_velocities_[i]);
  }
  
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
InverseJacobianController::on_export_reference_interfaces()
{

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  
//   // Export reference interfaces for chained mode
  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name(), "x/velocity", &ref_x_velocity_));
  
  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name(), "y/velocity", &ref_y_velocity_));
      
  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name(), "phi/velocity", &ref_phi_velocity_));

  return reference_interfaces;
}

std::vector<hardware_interface::StateInterface>
InverseJacobianController::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // No state interfaces exported
  return state_interfaces;
}


controller_interface::return_type InverseJacobianController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Get velocity command from subscriber if not in chained mode
  if (!is_chained_mode_)
  {
    auto velocity_command = velocity_command_ptr_.readFromRT();
    if (velocity_command && *velocity_command)
    {
      ref_x_velocity_ = (*velocity_command)->linear.x;
      ref_y_velocity_ = (*velocity_command)->linear.y;
      ref_phi_velocity_ = (*velocity_command)->angular.z;
    }
  }
  // In chained mode, reference values are set by the upstream controller
  
  return controller_interface::return_type::OK;
}
void InverseJacobianController::initializeInverseJacobianMatrix()
{
  // Initialize the inverse Jacobian matrix based on your Matlab code:
  // J = (1/r)*[+1 -1 -(b + a); +1 1 -(b + a); -1 1 -(b + a); -1 -1 -(b + a)]
  
  double inv_radius = 1.0 / WHEEL_RADIUS;
  double angular_coeff = -(HALF_WIDTH + HALF_LENGTH);
  
  // Row 0: Front left wheel
  inverse_jacobian_matrix_[0][0] = inv_radius * 1.0;  // x coefficient
  inverse_jacobian_matrix_[0][1] = inv_radius * 1.0;   // y coefficient
  inverse_jacobian_matrix_[0][2] = inv_radius * angular_coeff; // phi coefficient
  
  // Row 1: Front right wheel
  inverse_jacobian_matrix_[1][0] = inv_radius * -1.0;   // x coefficient
  inverse_jacobian_matrix_[1][1] = inv_radius * 1.0;   // y coefficient
  inverse_jacobian_matrix_[1][2] = inv_radius * angular_coeff; // phi coefficient
  
  // Row 2: Rear left wheel
  inverse_jacobian_matrix_[2][0] = inv_radius * -1.0;   // x coefficient
  inverse_jacobian_matrix_[2][1] = inv_radius * -1.0;  // y coefficient
  inverse_jacobian_matrix_[2][2] = inv_radius * angular_coeff; // phi coefficient
  
  // Row 3: Rear right wheel
  inverse_jacobian_matrix_[3][0] = inv_radius * 1.0;  // x coefficient
  inverse_jacobian_matrix_[3][1] = inv_radius * -1.0;  // y coefficient
  inverse_jacobian_matrix_[3][2] = inv_radius * angular_coeff; // phi coefficient
  
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Inverse Jacobian matrix initialized with a=%.3f, b=%.3f, r=%.3f",
    HALF_LENGTH, HALF_WIDTH, WHEEL_RADIUS);
}



void InverseJacobianController::computeWheelVelocities()
{
  // Apply inverse Jacobian transformation
  // wheel_velocities = inverse_jacobian * [x_vel; y_vel; phi_vel]
  
  for (size_t i = 0; i < 4; ++i)
  {
    wheel_velocities_[i] = 
      inverse_jacobian_matrix_[i][0] * ref_x_velocity_ +
      inverse_jacobian_matrix_[i][1] * ref_y_velocity_ +
      inverse_jacobian_matrix_[i][2] * ref_phi_velocity_;
  }
}

// Velocity command callback
void InverseJacobianController::velocityCommandCallback(
  const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  velocity_command_ptr_.writeFromNonRT(msg);
}

}  // namespace mecanum_jacobian

PLUGINLIB_EXPORT_CLASS(
  mecanum_jacobian::InverseJacobianController, controller_interface::ChainableControllerInterface)