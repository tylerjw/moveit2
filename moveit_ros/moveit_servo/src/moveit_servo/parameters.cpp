/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author    : Adam Pettinger, Tyler Weaver
   Desc      : Declares, loads, and checks ServoParameters for Servo
   Title     : servo_parameters.cpp
   Project   : moveit_servo
   Created   : 07/02/2020
*/

#include <parameter_set/parameter_set.hpp>
#include <parameter_set/validate_parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/parameters.hpp>

using parameter_set::ParameterDescriptorBuilder;
using rcl_interfaces::msg::ParameterType;
using rclcpp::ParameterValue;

namespace moveit_servo
{
namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_parameters");

}  // namespace

bool ServoParameters::declare(ParameterSetFactory* parameter_set_factory,
                              const NodeParametersInterface::SharedPtr& node_parameters)
{
  auto ns = getNamespace();
  node_parameters->declare_parameter(
      ns + ".use_gazebo", ParameterValue(use_gazebo),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_BOOL)
          .description("Whether the robot is started in a Gazebo simulation environment"));

  node_parameters->declare_parameter(ns + ".status_topic", ParameterValue(status_topic),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_STRING)
                                         .description("Topic to publish status of servo on")
                                         .additional_constraints("Must be a valid topic name"));
  parameter_set_factory->registerValidateFunction(ns + ".status_topic", &parameter_set::validate::topic_name);

  node_parameters->declare_parameter(
      ns + ".robot_description_name", ParameterValue(robot_description_name),
      ParameterDescriptorBuilder().type(ParameterType::PARAMETER_STRING).description("Robot description parameter"));

  // incomming commands parameters
  node_parameters->declare_parameter(ns + ".cartesian_command_in_topic", ParameterValue(cartesian_command_in_topic),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_STRING)
                                         .description("Topic to subscribe to for cartesian commands")
                                         .additional_constraints("Must be a valid topic name"));
  parameter_set_factory->registerValidateFunction(ns + ".cartesian_command_in_topic",
                                                  &parameter_set::validate::topic_name);

  node_parameters->declare_parameter(ns + ".joint_command_in_topic", ParameterValue(joint_command_in_topic),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_STRING)
                                         .description("Topic to subscribe to for joint commands")
                                         .additional_constraints("Must be a valid topic name"));
  parameter_set_factory->registerValidateFunction(ns + ".joint_command_in_topic", &parameter_set::validate::topic_name);

  node_parameters->declare_parameter(
      ns + ".robot_link_command_frame", ParameterValue(robot_link_command_frame),
      ParameterDescriptorBuilder().type(ParameterType::PARAMETER_STRING).description("Robot link command frame"));

  node_parameters->declare_parameter(
      ns + ".command_in_type", ParameterValue(command_in_type),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_STRING)
          .description("'unitless'> in the range [-1:1], as if from joystick. 'speed_units'> cmds are in m/s and rad/s")
          .additional_constraints("Must be either 'unitless' or 'speed_units'"));
  parameter_set_factory->registerValidateFunction(ns + ".command_in_type", [](auto parameter) {
    return parameter_set::validate::in_string_set(parameter, { "unitless", "speed_units" });
  });

  node_parameters->declare_parameter(ns + ".scale.linear", ParameterValue(linear_scale),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_DOUBLE)
                                         .description("Max linear velocity. Meters per publish_period. Unit is [m/s]. "
                                                      "Only used for Cartesian commands."));

  node_parameters->declare_parameter(ns + ".scale.rotational", ParameterValue(rotational_scale),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_DOUBLE)
                                         .description("Max angular velocity. Rads per publish_period. Unit is [rad/s]. "
                                                      "Only used for Cartesian commands."));

  node_parameters->declare_parameter(
      ns + ".scale.joint", ParameterValue(joint_scale),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_DOUBLE)
          .description("Max joint angular/linear velocity. Rads or Meters per publish period. Only used for joint "
                       "commands on joint_command_in_topic."));

  // Properties of outgoing commands
  node_parameters->declare_parameter(ns + ".command_out_topic", ParameterValue(command_out_topic),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_STRING)
                                         .description("Publish outgoing commands here")
                                         .additional_constraints("Must be a valid topic name"));
  parameter_set_factory->registerValidateFunction(ns + ".command_out_topic", &parameter_set::validate::topic_name);

  node_parameters->declare_parameter(ns + ".publish_period", ParameterValue(publish_period),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_DOUBLE)
                                         .description("1/Nominal publish rate [seconds]")
                                         .additional_constraints("Must be greater than 0")
                                         .floating_point_range(1e-9));

  node_parameters->declare_parameter(
      ns + ".command_out_type", ParameterValue(command_out_type),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_STRING)
          .description("What type of topic does your robot driver expect?")
          .additional_constraints(
              "Must be either in the set ['std_msgs/Float64MultiArray', 'trajectory_msgs/JointTrajectory']"));
  parameter_set_factory->registerValidateFunction(ns + ".command_out_type", [](auto parameter) {
    return parameter_set::validate::in_string_set(parameter,
                                                  { "std_msgs/Float64MultiArray", "trajectory_msgs/JointTrajectory" });
  });

  node_parameters->declare_parameter(
      ns + ".publish_joint_positions", ParameterValue(publish_joint_positions),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_BOOL)
          .description("What to publish? Can save some bandwidth as most robots only require positions or velocities"));

  node_parameters->declare_parameter(
      ns + ".publish_joint_velocities", ParameterValue(publish_joint_velocities),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_BOOL)
          .description("What to publish? Can save some bandwidth as most robots only require positions or velocities"));

  node_parameters->declare_parameter(
      ns + ".publish_joint_accelerations", ParameterValue(publish_joint_accelerations),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_BOOL)
          .description("What to publish? Can save some bandwidth as most robots only require positions or velocities"));

  node_parameters->declare_parameter(ns + ".low_latency_mode", ParameterValue(low_latency_mode),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_BOOL)
                                         .description("Set this to true to publish as soon as an incoming Twist "
                                                      "command is received (publish_period is ignored)"));

  // Incoming Joint State properties
  node_parameters->declare_parameter(ns + ".joint_topic", ParameterValue(joint_topic),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_STRING)
                                         .description("Topic to subscribe to for joint positions")
                                         .additional_constraints("Must be a valid topic name"));
  parameter_set_factory->registerValidateFunction(ns + ".joint_topic", &parameter_set::validate::topic_name);

  node_parameters->declare_parameter(
      ns + ".low_pass_filter_coeff", ParameterValue(low_pass_filter_coeff),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_DOUBLE)
          .description("Larger --> trust the filtered data more, trust the measurements less.")
          .floating_point_range(1.0 + 1e-4));

  // MoveIt properties
  node_parameters->declare_parameter(
      ns + ".move_group_name", ParameterValue(move_group_name),
      ParameterDescriptorBuilder().type(ParameterType::PARAMETER_STRING).description("Often 'manipulator' or 'arm'"));

  node_parameters->declare_parameter(ns + ".planning_frame", ParameterValue(planning_frame),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_STRING)
                                         .description("The MoveIt planning frame. Often 'base_link' or 'world'"));

  node_parameters->declare_parameter(ns + ".ee_frame_name", ParameterValue(ee_frame_name),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_STRING)
                                         .description("The name of the end effector link, used to return the EE pose"));

  // Stopping behaviour
  node_parameters->declare_parameter(ns + ".incoming_command_timeout", ParameterValue(incoming_command_timeout),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_DOUBLE)
                                         .description("Stop servoing if X seconds elapse without a new command")
                                         .floating_point_range(1e-9));

  node_parameters->declare_parameter(
      ns + ".num_outgoing_halt_msgs_to_publish", ParameterValue(num_outgoing_halt_msgs_to_publish),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_INTEGER)
          .description("If 0, republish commands forever even if the robot is stationary. Otherwise, specify num. to "
                       "publish.  Important because ROS may drop some messages and we need the robot to halt reliably.")
          .integer_range(0));

  // Configure handling of singularities and joint limits
  node_parameters->declare_parameter(
      ns + ".lower_singularity_threshold", ParameterValue(lower_singularity_threshold),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_DOUBLE)
          .description("Start decelerating when the condition number hits this (close to singularity)")
          .floating_point_range(1e-9));

  node_parameters->declare_parameter(ns + ".hard_stop_singularity_threshold",
                                     ParameterValue(hard_stop_singularity_threshold),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_DOUBLE)
                                         .description("Stop when the condition number hits this")
                                         .floating_point_range(1e-9));

  node_parameters->declare_parameter(
      ns + ".joint_limit_margin", ParameterValue(joint_limit_margin),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_DOUBLE)
          .description("added as a buffer to joint limits [radians]. If moving quickly, make this larger.")
          .floating_point_range(0));

  // Collision checking
  node_parameters->declare_parameter(
      ns + ".check_collisions", ParameterValue(check_collisions),
      ParameterDescriptorBuilder().type(ParameterType::PARAMETER_BOOL).description("Check collisions?"));

  node_parameters->declare_parameter(
      ns + ".collision_check_rate", ParameterValue(collision_check_rate),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_DOUBLE)
          .description("[Hz] Collision-checking can easily bog down a CPU if done too often")
          .floating_point_range(1e-9));

  node_parameters->declare_parameter(
      ns + ".collision_check_type", ParameterValue(collision_check_type),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_STRING)
          .description(
              "'threshold_distance' begins slowing down when nearer than a specified distance. Good if you want to "
              "tune collision thresholds manually.  'stop_distance' stops if a collision is nearer than the worst-case "
              "stopping distance and the distance is decreasing. Requires joint acceleration limits.")
          .additional_constraints("Must be either in the set ['threshold_distance', 'stop_distance']"));
  parameter_set_factory->registerValidateFunction(ns + ".collision_check_type", [](auto parameter) {
    return parameter_set::validate::in_string_set(parameter, { "threshold_distance", "stop_distance" });
  });

  node_parameters->declare_parameter(
      ns + ".self_collision_proximity_threshold", ParameterValue(self_collision_proximity_threshold),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_DOUBLE)
          .description("'threshold_distance': Start decelerating when a self-collision is this far [m]")
          .floating_point_range(1e-9));

  node_parameters->declare_parameter(
      ns + ".scene_collision_proximity_threshold", ParameterValue(scene_collision_proximity_threshold),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_DOUBLE)
          .description("'threshold_distance': Start decelerating when a scene collision is this far [m]")
          .floating_point_range(1e-9));

  node_parameters->declare_parameter(
      ns + ".collision_distance_safety_factor", ParameterValue(collision_distance_safety_factor),
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_DOUBLE)
          .description("'stop_distance': Must be >= 1. A large safety factor is recommended to account for latency")
          .floating_point_range(1.0));

  node_parameters->declare_parameter(ns + ".min_allowable_collision_distance",
                                     ParameterValue(min_allowable_collision_distance),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_DOUBLE)
                                         .description("'stop_distance': Stop if a collision is closer than this [m]")
                                         .floating_point_range(1e-9));

  return true;
}

bool ServoParameters::get(const NodeParametersInterface::SharedPtr& node_parameters)
{
  auto ns = getNamespace();

  use_gazebo = node_parameters->get_parameter(ns + ".use_gazebo").as_bool();
  status_topic = node_parameters->get_parameter(ns + ".status_topic").as_string();
  robot_description_name = node_parameters->get_parameter(ns + ".robot_description_name").as_string();

  // incomming commands parameters
  cartesian_command_in_topic = node_parameters->get_parameter(ns + ".cartesian_command_in_topic").as_string();
  joint_command_in_topic = node_parameters->get_parameter(ns + ".joint_command_in_topic").as_string();
  robot_link_command_frame = node_parameters->get_parameter(ns + ".robot_link_command_frame").as_string();
  command_in_type = node_parameters->get_parameter(ns + ".command_in_type").as_string();
  linear_scale = node_parameters->get_parameter(ns + ".scale.linear").as_double();
  rotational_scale = node_parameters->get_parameter(ns + ".scale.rotational").as_double();
  joint_scale = node_parameters->get_parameter(ns + ".scale.joint").as_double();

  // Properties of outgoing commands
  command_out_topic = node_parameters->get_parameter(ns + ".command_out_topic").as_string();
  publish_period = node_parameters->get_parameter(ns + ".publish_period").as_double();
  command_out_topic = node_parameters->get_parameter(ns + ".command_out_topic").as_string();
  publish_joint_positions = node_parameters->get_parameter(ns + ".publish_joint_positions").as_bool();
  publish_joint_velocities = node_parameters->get_parameter(ns + ".publish_joint_velocities").as_bool();
  publish_joint_accelerations = node_parameters->get_parameter(ns + ".publish_joint_accelerations").as_bool();
  low_latency_mode = node_parameters->get_parameter(ns + ".low_latency_mode").as_bool();

  // Incoming Joint State properties
  joint_topic = node_parameters->get_parameter(ns + ".joint_topic").as_string();
  low_pass_filter_coeff = node_parameters->get_parameter(ns + ".low_pass_filter_coeff").as_double();

  // MoveIt properties
  move_group_name = node_parameters->get_parameter(ns + ".move_group_name").as_string();
  planning_frame = node_parameters->get_parameter(ns + ".planning_frame").as_string();
  ee_frame_name = node_parameters->get_parameter(ns + ".ee_frame_name").as_string();

  // Stopping behaviour
  incoming_command_timeout = node_parameters->get_parameter(ns + ".incoming_command_timeout").as_double();
  num_outgoing_halt_msgs_to_publish =
      node_parameters->get_parameter(ns + ".num_outgoing_halt_msgs_to_publish").as_int();

  // Configure handling of singularities and joint limits
  lower_singularity_threshold = node_parameters->get_parameter(ns + ".lower_singularity_threshold").as_double();
  hard_stop_singularity_threshold = node_parameters->get_parameter(ns + ".hard_stop_singularity_threshold").as_double();
  joint_limit_margin = node_parameters->get_parameter(ns + ".joint_limit_margin").as_double();

  // Collision checking
  check_collisions = node_parameters->get_parameter(ns + ".check_collisions").as_bool();
  collision_check_rate = node_parameters->get_parameter(ns + ".collision_check_rate").as_double();
  collision_check_type = node_parameters->get_parameter(ns + ".collision_check_type").as_string();
  self_collision_proximity_threshold =
      node_parameters->get_parameter(ns + ".self_collision_proximity_threshold").as_double();
  scene_collision_proximity_threshold =
      node_parameters->get_parameter(ns + ".scene_collision_proximity_threshold").as_double();
  collision_distance_safety_factor =
      node_parameters->get_parameter(ns + ".collision_distance_safety_factor").as_double();
  min_allowable_collision_distance =
      node_parameters->get_parameter(ns + ".min_allowable_collision_distance").as_double();

  // Begin input checking
  is_valid = true;
  if (hard_stop_singularity_threshold <= lower_singularity_threshold)
  {
    RCLCPP_WARN(LOGGER, "Parameter 'hard_stop_singularity_threshold' "
                        "should be greater than 'lower_singularity_threshold.' "
                        "Check yaml file.");
    is_valid = false;
  }
  if (!publish_joint_positions && !publish_joint_velocities && !publish_joint_accelerations)
  {
    RCLCPP_WARN(LOGGER, "At least one of publish_joint_positions / "
                        "publish_joint_velocities / "
                        "publish_joint_accelerations must be true. Check "
                        "yaml file.");
    is_valid = false;
  }
  if ((command_out_type == "std_msgs/Float64MultiArray") && publish_joint_positions && publish_joint_velocities)
  {
    RCLCPP_WARN(LOGGER, "When publishing a std_msgs/Float64MultiArray, "
                        "you must select positions OR velocities.");
    is_valid = false;
  }
  // Collision checking
  if (scene_collision_proximity_threshold < self_collision_proximity_threshold)
  {
    RCLCPP_WARN(LOGGER, "Parameter 'self_collision_proximity_threshold' should probably be less "
                        "than or equal to 'scene_collision_proximity_threshold'. Check yaml file.");
  }
  if (joint_limit_margin < 0.)
  {
    RCLCPP_WARN(LOGGER, "Parameter 'joint_limit_margin' should usually be  greater than or equal to zero, "
                        "although negative values can be used if the specified joint limits are actually soft. "
                        "Check yaml file.");
  }

  return is_valid;
}

}  // namespace moveit_servo
