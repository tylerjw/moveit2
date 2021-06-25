/*******************************************************************************
 *      Title     : pose_tracking_parameters.h
 *      Project   : moveit_servo
 *      Created   : 6/25/2021
 *      Author    : Tyler Weaver
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Inc.
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

using parameter_set::ParameterSet;
using parameter_set::ParameterSetFactory;
using rclcpp::node_interfaces::NodeParametersInterface;

namespace pose_tracking
{
// Size of queues used in ros pub/sub/service
constexpr size_t ROS_QUEUE_SIZE = 2;

struct PIDConfig
{
  // Default values
  double k_p = 1;
  double k_i = 0;
  double k_d = 0;
};

// ROS params to be read. See the yaml file in /config for a description of each.
struct PoseTrackingParameters : public ParameterSet
{
  using ParameterSet::ParameterSet;

  double windup_limit = 0.1;
  PIDConfig x_pid;
  PIDConfig y_pid;
  PIDConfig z_pid;
  PIDConfig angular_pid;

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

}  // namespace pose_tracking
