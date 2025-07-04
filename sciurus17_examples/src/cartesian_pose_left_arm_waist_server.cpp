#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sciurus17_interfaces/action/set_pose.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class CartesianPoseLeftArmWaistServer : public rclcpp::Node
{
public:
  using SetPose = sciurus17_interfaces::action::SetPose;
  using GoalHandleSetPose = rclcpp_action::ServerGoalHandle<SetPose>;

  CartesianPoseLeftArmWaistServer(const rclcpp::Node::SharedPtr & move_group_node) : Node("cartesian_pose_left_arm_waist_node"), move_group_node_(move_group_node)
  {
    action_server_ = rclcpp_action::create_server<SetPose>(
      this,
      "cartesian_pose_left_arm_waist",  // アクション名
      std::bind(&CartesianPoseLeftArmWaistServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CartesianPoseLeftArmWaistServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&CartesianPoseLeftArmWaistServer::handle_accepted, this, std::placeholders::_1)
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
    const auto & pose_goal = goal->target_pose;

    RCLCPP_INFO(this->get_logger(), "Received Goal Pose:");
    RCLCPP_INFO(this->get_logger(), "  position: [x=%.3f, y=%.3f, z=%.3f]",
                pose_goal.position.x, pose_goal.position.y, pose_goal.position.z);
    RCLCPP_INFO(this->get_logger(), "  orientation: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
                pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w);

    // MoveGroupInterface の準備
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node_, "l_arm_waist_group");
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);

    // 現在のPoseを取得（エンドエフェクタ名を指定）
    geometry_msgs::msg::Pose pose_start = move_group.getCurrentPose("l_link7").pose;

    // 直線移動のwaypointsを定義
    std::vector<geometry_msgs::msg::Pose> waypoints{pose_start, pose_goal};

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    bool success = false;
    if (fraction > 0.9) {
        RCLCPP_INFO(this->get_logger(), "Cartesian path planning succeeded (%.2f%%). Executing...", fraction * 100.0);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        auto result_code = move_group.execute(plan);
        success = (result_code == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
        RCLCPP_WARN(this->get_logger(), "Cartesian path planning failed (%.2f%%).", fraction * 100.0);
    }

    // 結果を返す
    auto result = std::make_shared<SetPose::Result>();
    result->success = success;
    result->message = success ? "Cartesian motion executed successfully." : "Cartesian planning or execution failed.";
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
  auto action_node = std::make_shared<CartesianPoseLeftArmWaistServer>(move_group_node);
  

  // 並列実行
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.add_node(action_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}