#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sciurus17_interfaces/action/set_pose.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class SetEndEffectorPoseLeftArmServer : public rclcpp::Node
{
public:
  using SetPose = sciurus17_interfaces::action::SetPose;
  using GoalHandleSetPose = rclcpp_action::ServerGoalHandle<SetPose>;

  SetEndEffectorPoseLeftArmServer(const rclcpp::Node::SharedPtr & move_group_node) : Node("set_end_effector_pose_left_arm_node"), move_group_node_(move_group_node)
  {
    action_server_ = rclcpp_action::create_server<SetPose>(
      this,
      "set_end_effector_pose_left_arm",  // アクション名
      std::bind(&SetEndEffectorPoseLeftArmServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SetEndEffectorPoseLeftArmServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SetEndEffectorPoseLeftArmServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Action server started.");
  }

private:
  rclcpp_action::Server<SetPose>::SharedPtr action_server_;
  rclcpp::Node::SharedPtr move_group_node_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const SetPose::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSetPose>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSetPose> goal_handle)
  {

  std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    const auto & pose = goal->target_pose;

    RCLCPP_INFO(this->get_logger(), "Received Goal Pose:");
    RCLCPP_INFO(this->get_logger(), "  position: [x=%.3f, y=%.3f, z=%.3f]",
                pose.position.x, pose.position.y, pose.position.z);
    RCLCPP_INFO(this->get_logger(), "  orientation: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    // MoveGroupInterface を生成
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node_, "l_arm_group");

    // MoveItの各種設定
    move_group.setMaxVelocityScalingFactor(0.2);            // 速度スケーリング
    move_group.setMaxAccelerationScalingFactor(0.2);        // 加速度スケーリング


    // ゴール姿勢をセット
    move_group.setPoseTarget(pose, "l_link7");

    // 計画を立てる
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // 成功したら実行
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing trajectory...");
      auto result_code = move_group.execute(plan);
      success = (result_code == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
      RCLCPP_WARN(this->get_logger(), "Planning failed.");
    }

    // 結果を返す
    auto result = std::make_shared<SetPose::Result>();
    result->success = success;
    result->message = success ? "Motion executed successfully." : "Motion planning or execution failed.";
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Result sent: %s", result->message.c_str());
  }).detach();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // MoveIt用のノードを作成
  auto move_group_node = rclcpp::Node::make_shared("move_group_node");

  // アクションサーバーノードを引数付きで生成
  auto action_node = std::make_shared<SetEndEffectorPoseLeftArmServer>(move_group_node);
  

  // 並列実行
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.add_node(action_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}