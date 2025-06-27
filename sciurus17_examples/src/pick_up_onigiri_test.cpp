// Copyright 2023 RT Corporation
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

// Reference:
// https://github.com/ros-planning/moveit2_tutorials/blob/humble/doc/
// examples/move_group_interface/src/move_group_interface_tutorial.cpp

#include <cmath>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "pose_presets.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_left_arm_waist");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  // 右腕と腰制御用MoveGroupInterface
  MoveGroupInterface move_group_arm(move_group_arm_node, "l_arm_waist_group");
  // 駆動速度の調整
  move_group_arm.setMaxVelocityScalingFactor(0.1);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(0.1);  // Set 0.0 ~ 1.0

  // 右グリッパ制御用MoveGroupInterface
  MoveGroupInterface move_group_gripper(move_group_gripper_node, "l_gripper_group");
  // 駆動速度の調整
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  /////////////////////////////////////////////////////////////////////////////////////

  // グリッパの開閉角
  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();
  const double GRIPPER_CLOSE = 0.0;
  const double GRIPPER_OPEN = angles::from_degrees(-50.0);
  const double GRIPPER_GRASP = angles::from_degrees(0.0);

  // 物体を掴む位置
  const double PICK_POSITION_X = -0.35;
  const double PICK_POSITION_Y = 0.4;
  const double PICK_POSITION_Z = 0.1;

  // 物体を置く位置
  const double PLACE_POSITION_X = 0.4;
  const double PLACE_POSITION_Y = 0.0;
  const double PLACE_POSITION_Z = 0.1;

  // 物体を持ち上げる高さ
  const double LIFTING_HEIGHT = 0.4;

  // SRDFに定義されている"l_arm_waist_init_pose"の姿勢にする
  move_group_arm.setNamedTarget("l_arm_waist_init_pose");
  move_group_arm.move();

  // 何かを掴んでいた時のためにハンドを開く
  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // 可動範囲を制限する
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "arm_constraints";

  // 腰軸の可動範囲を制限する
  //moveit_msgs::msg::JointConstraint joint_constraint;
  //joint_constraint.joint_name = "waist_yaw_joint";
  //joint_constraint.position = 0.0;
  //joint_constraint.tolerance_above = angles::from_degrees(45);
  //joint_constraint.tolerance_below = angles::from_degrees(45);
  //joint_constraint.weight = 1.0;
  //constraints.joint_constraints.push_back(joint_constraint);

  move_group_arm.setPathConstraints(constraints);
  
  // ボックスの方向を向く
  auto arm_joint_values = move_group_arm.getCurrentJointValues();
  arm_joint_values[0] = angles::from_degrees(90.0);
  move_group_arm.setJointValueTarget(arm_joint_values);

  // 物体の上に腕を伸ばす
  move_group_arm.setPoseTarget(
    pose_presets::left_arm_downward(PICK_POSITION_X, PICK_POSITION_Y, LIFTING_HEIGHT));
  move_group_arm.move();

  // ハンドを開く
  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // 掴みに行く
  // move_group_arm.setPoseTarget(
  //   pose_presets::left_arm_downward(PICK_POSITION_X, PICK_POSITION_Y, PICK_POSITION_Z));
  // move_group_arm.move();

  // 直線移動で降下
  geometry_msgs::msg::Pose pose_top = pose_presets::left_arm_downward(
    PICK_POSITION_X, PICK_POSITION_Y, LIFTING_HEIGHT);
  geometry_msgs::msg::Pose pose_bottom = pose_presets::left_arm_downward(
    PICK_POSITION_X, PICK_POSITION_Y, PICK_POSITION_Z);
  std::vector<geometry_msgs::msg::Pose> waypoints{pose_top, pose_bottom};
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_arm.computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction > 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_arm.execute(plan);
  }

  // ハンドを閉じる
  gripper_joint_values[0] = GRIPPER_GRASP;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // 持ち上げる
  move_group_arm.setPoseTarget(
    pose_presets::left_arm_downward(PICK_POSITION_X, PICK_POSITION_Y, LIFTING_HEIGHT));
  move_group_arm.move();

  // 棚を向く
  arm_joint_values = move_group_arm.getCurrentJointValues();
  arm_joint_values[0] = angles::from_degrees(0.0);
  move_group_arm.setJointValueTarget(arm_joint_values);

  // 移動する
  move_group_arm.setPoseTarget(
    pose_presets::left_arm_downward(PLACE_POSITION_X, PLACE_POSITION_Y, LIFTING_HEIGHT));
  move_group_arm.move();

  // 下ろす
  // move_group_arm.setPoseTarget(
  //   pose_presets::left_arm_downward(PLACE_POSITION_X, PLACE_POSITION_Y, PLACE_POSITION_Z));
  // move_group_arm.move();

  // 直線で上昇
  pose_top = pose_presets::left_arm_downward(
    PLACE_POSITION_X, PLACE_POSITION_Y, LIFTING_HEIGHT);
  pose_bottom = pose_presets::left_arm_downward(
    PLACE_POSITION_X, PLACE_POSITION_Y, PLACE_POSITION_Z);
  waypoints = {pose_top, pose_bottom};
  trajectory = moveit_msgs::msg::RobotTrajectory(); // 初期化
  fraction = move_group_arm.computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction > 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_arm.execute(plan);
  }

  // ハンドを開く
  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // 少しだけハンドを持ち上げる
  move_group_arm.setPoseTarget(
    pose_presets::left_arm_downward(PLACE_POSITION_X, PLACE_POSITION_Y, LIFTING_HEIGHT));
  move_group_arm.move();

  // 可動範囲の制限を解除
  move_group_arm.clearPathConstraints();

  // SRDFに定義されている"r_arm_waist_init_pose"の姿勢にする
  move_group_arm.setNamedTarget("l_arm_waist_init_pose");
  move_group_arm.move();

  // ハンドを閉じる
  gripper_joint_values[0] = GRIPPER_CLOSE;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // SRDFに定義されている"l_arm_waist_init_pose"の姿勢にする
  move_group_arm.setNamedTarget("l_arm_waist_init_pose");
  move_group_arm.move();

  rclcpp::shutdown();
  return 0;
}
