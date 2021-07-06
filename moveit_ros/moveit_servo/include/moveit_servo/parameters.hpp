/*******************************************************************************
 *      Title     : servo_parameters.h
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#pragma once

#include <parameter_set/parameter_set.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

using parameter_set::ParameterSet;
using parameter_set::ParameterSetFactory;
using rclcpp::node_interfaces::NodeParametersInterface;

namespace moveit_servo
{
// Size of queues used in ros pub/sub/service
constexpr size_t ROS_QUEUE_SIZE = 2;

// ROS params to be read. See the yaml file in /config for a description of each.
struct ServoParameters : public ParameterSet
{
  using ParameterSet::ParameterSet;

  // ROS Parameters
  // Note that all of these are effectively const because the only way to create one of these
  //  is as a shared_ptr to a constant struct.
  bool use_gazebo = false;
  std::string status_topic = "~/status";
  std::string robot_description_name = "robot_description";

  // Properties of incoming commands
  std::string cartesian_command_in_topic = "~/delta_twist_cmds";
  std::string joint_command_in_topic = "~/delta_joint_cmds";
  std::string robot_link_command_frame = "";
  std::string command_in_type = "unitless";
  double linear_scale = 0.4;
  double rotational_scale = 0.8;
  double joint_scale = 0.5;

  // Properties of outgoing commands
  std::string command_out_topic = "";
  double publish_period = 0.034;
  std::string command_out_type = "trajectory_msgs/JointTrajector";
  bool publish_joint_positions = true;
  bool publish_joint_velocities = false;
  bool publish_joint_accelerations = false;

  // Incoming Joint State properties
  std::string joint_topic = "/joint_states";
  double low_pass_filter_coeff = 2.0;
  // MoveIt properties
  std::string move_group_name = "arm";
  std::string planning_frame = "world";
  std::string ee_frame_name = "";

  // Stopping behaviour
  double incoming_command_timeout = 0.1;
  int num_outgoing_halt_msgs_to_publish = 4;

  // Configure handling of singularities and joint limits
  double lower_singularity_threshold = 17.0;
  double hard_stop_singularity_threshold = 30.0;
  double joint_limit_margin = 0.1;
  bool low_latency_mode = false;

  // Collision checking
  bool check_collisions = true;
  double collision_check_rate = 10.0;
  std::string collision_check_type = "threshold_distance";
  double self_collision_proximity_threshold = 0.01;
  double scene_collision_proximity_threshold = 0.02;
  double collision_distance_safety_factor = 1000.0;
  double min_allowable_collision_distance = 0.01;

  // Set to false if get detects an error in configuration
  bool is_valid = true;

  /**
   * @brief      Declare the parameters, called by ParameterSetFactory
   *
   * @param[in]  parameter_set_factory  The parameter set factory pointer
   * @param[in]  node_parameters        The node parameters interface
   *
   * @return     true on success
   */
  bool declare(ParameterSetFactory* parameter_set_factory,
               const NodeParametersInterface::SharedPtr& node_parameters) override;

  /**
   * @brief      Get the parameters, called by ParameterSetFactory
   *
   * @param[in]  node_parameters        The node parameters interface
   *
   * @return     true on success
   */
  bool get(const NodeParametersInterface::SharedPtr& node_parameters) override;
};

}  // namespace moveit_servo
