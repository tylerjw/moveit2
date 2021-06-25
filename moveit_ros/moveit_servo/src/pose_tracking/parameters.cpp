/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
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

/* Author    : Tyler Weaver
   Desc      : Declares, gets, and validates parameters for pose tracking
   Title     : pose_tracking.cpp
   Project   : pose_tracking
   Created   : 06/25/2021
*/

#include <parameter_set/parameter_set.hpp>
#include <parameter_set/validate_parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pose_tracking/parameters.hpp>

using parameter_set::ParameterDescriptorBuilder;
using rcl_interfaces::msg::ParameterType;
using rclcpp::ParameterValue;

namespace pose_tracking
{
namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pose_tracking.parameters");

}  // namespace

bool PoseTrackingParameters::declare(ParameterSetFactory* parameter_set_factory,
                                     const NodeParametersInterface::SharedPtr& node_parameters)
{
  auto ns = getNamespace();
  node_parameters->declare_parameter(ns + ".windup_limit", ParameterValue(windup_limit),
                                     ParameterDescriptorBuilder()
                                         .type(ParameterType::PARAMETER_DOUBLE)
                                         .description("Maximum value of error integral for all PID controllers"));

  node_parameters->declare_parameter(ns + ".x_proportional_gain", ParameterValue(x_pid.k_p),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));
  node_parameters->declare_parameter(ns + ".x_integral_gain", ParameterValue(x_pid.k_i),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));
  node_parameters->declare_parameter(ns + ".x_derivative_gain", ParameterValue(x_pid.k_d),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));

  node_parameters->declare_parameter(ns + ".y_proportional_gain", ParameterValue(y_pid.k_p),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));
  node_parameters->declare_parameter(ns + ".y_integral_gain", ParameterValue(y_pid.k_i),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));
  node_parameters->declare_parameter(ns + ".y_derivative_gain", ParameterValue(y_pid.k_d),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));

  node_parameters->declare_parameter(ns + ".z_proportional_gain", ParameterValue(z_pid.k_p),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));
  node_parameters->declare_parameter(ns + ".z_integral_gain", ParameterValue(z_pid.k_i),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));
  node_parameters->declare_parameter(ns + ".z_derivative_gain", ParameterValue(z_pid.k_d),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));

  node_parameters->declare_parameter(ns + ".angular_proportional_gain", ParameterValue(angular_pid.k_p),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));
  node_parameters->declare_parameter(ns + ".angular_integral_gain", ParameterValue(angular_pid.k_i),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));
  node_parameters->declare_parameter(ns + ".angular_derivative_gain", ParameterValue(angular_pid.k_d),
                                     ParameterDescriptorBuilder().type(ParameterType::PARAMETER_DOUBLE));

  return true;
}

bool PoseTrackingParameters::get(const NodeParametersInterface::SharedPtr& node_parameters)
{
  auto ns = getNamespace();

  windup_limit = node_parameters->get_parameter(ns + ".windup_limit").as_double();

  x_pid.k_p = node_parameters->get_parameter(ns + ".x_proportional_gain").as_double();
  x_pid.k_i = node_parameters->get_parameter(ns + ".x_integral_gain").as_double();
  x_pid.k_d = node_parameters->get_parameter(ns + ".x_derivative_gain").as_double();

  y_pid.k_p = node_parameters->get_parameter(ns + ".y_proportional_gain").as_double();
  y_pid.k_i = node_parameters->get_parameter(ns + ".y_integral_gain").as_double();
  y_pid.k_d = node_parameters->get_parameter(ns + ".y_derivative_gain").as_double();

  z_pid.k_p = node_parameters->get_parameter(ns + ".z_proportional_gain").as_double();
  z_pid.k_i = node_parameters->get_parameter(ns + ".z_integral_gain").as_double();
  z_pid.k_d = node_parameters->get_parameter(ns + ".z_derivative_gain").as_double();

  angular_pid.k_p = node_parameters->get_parameter(ns + ".angular_proportional_gain").as_double();
  angular_pid.k_i = node_parameters->get_parameter(ns + ".angular_integral_gain").as_double();
  angular_pid.k_d = node_parameters->get_parameter(ns + ".angular_derivative_gain").as_double();

  return true;
}

}  // namespace pose_tracking
