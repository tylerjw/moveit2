/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/*      Title     : unit_test_servo_calcs.hpp
 *      Project   : moveit_servo
 *      Created   : 08/04/2020
 *      Author    : Adam Pettinger
 *      Desc      : Sets up test fixtures for unit testing ServoCalcs
 */

#pragma once

#include <gtest/gtest.h>
#include <moveit_servo/servo_calcs.h>

const std::vector<std::string> panda_joint_names_{"panda_finger_joint1", "panda_finger_joint2", "panda_joint1", "panda_joint2"
    "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

// subclassing and friending so we can access member varibles
class FriendServoCalcs : public moveit_servo::ServoCalcs
{
  FRIEND_TEST(ServoCalcsTestFixture, TestRemoveSingleDimension);
  FRIEND_TEST(ServoCalcsTestFixture, TestRemoveDriftDimensions);
  FRIEND_TEST(ServoCalcsTestFixture, TestEnforceControlDimensions);
  FRIEND_TEST(ServoCalcsTestFixture, TestCheckValidCommand);
  FRIEND_TEST(ServoCalcsTestFixture, TestApplyJointUpdate);
  FRIEND_TEST(ServoCalcsTestFixture, TestInsertRedundantPoints);
public:
  FriendServoCalcs(const rclcpp::Node::SharedPtr& node, const moveit_servo::ServoParametersPtr& parameters,
             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);
};

class ServoCalcsTestFixture : public ::testing::Test
{
public:
  ServoCalcsTestFixture();
  ~ServoCalcsTestFixture(){};
  void SetUp();
  void TearDown();

  void SetUpStateController();

protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<FriendServoCalcs> servo_calcs_;
  moveit_servo::ServoParametersPtr parameters_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};