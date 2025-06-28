#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sciurus17_interfaces/action/set_joint_values.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <angles/angles.h>

class SetJointValuesLeftArmWaistServer : public rclcpp::Node, public std::enable_shared_from_this<SetJointValuesLeftArmWaistServer>
{
public:
  using SetJointValues = sciurus17_interfaces::action::SetJointValues;
  using GoalHandle = rclcpp_action::ServerGoalHandle<SetJointValues>;

  SetJointValuesLeftArmWaistServer(const rclcpp::Node::SharedPtr & move_group_node) : Node("set_joint_values_left_arm_waist_server"), move_group_node_(move_group_node)
  {
    action_server_ = rclcpp_action::create_server<SetJointValues>(
      this,
      "set_joint_values_left_arm_waist",
      std::bind(&SetJointValuesLeftArmWaistServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SetJointValuesLeftArmWaistServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SetJointValuesLeftArmWaistServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "SetJointValuesLeftArmWaistServer is up.");
  }

private:
  rclcpp_action::Server<SetJointValues>::SharedPtr action_server_;
  rclcpp::Node::SharedPtr move_group_node_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const SetJointValues::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
  std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    const auto & target_joint_values = goal->joint_values;

    // MoveGroupInterface の作成
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node_, "l_arm_waist_group");

    // 現在のジョイント値を取得・表示
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    std::ostringstream oss;
    oss << "{joint_values: [";
    for (size_t i = 0; i < current_joint_values.size(); ++i) {
      oss << current_joint_values[i];
      if (i != current_joint_values.size() - 1) {
        oss << ", ";
      }
    }
    oss << "]}";
    RCLCPP_INFO(this->get_logger(), "Current joint values: %s", oss.str().c_str());

    // 関節数チェック（セーフティ）
    if (target_joint_values.size() != move_group.getJointNames().size()) {
      RCLCPP_ERROR(this->get_logger(), "Received joint value size (%zu) does not match expected (%zu)",
                   target_joint_values.size(),
                   move_group.getJointNames().size());

      auto result = std::make_shared<SetJointValues::Result>();
      result->success = false;
      result->message = "Invalid number of joint values received.";
      goal_handle->succeed(result);
      return;
    }

    // 目標値をセット
    move_group.setJointValueTarget(target_joint_values);

    // 計画＆実行
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing...");
      auto result_code = move_group.execute(plan);
      success = (result_code == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
      RCLCPP_WARN(this->get_logger(), "Motion planning failed.");
    }

    // 結果送信
    auto result = std::make_shared<SetJointValues::Result>();
    result->success = success;
    result->message = success ? "Joint motion executed successfully." : "Motion planning or execution failed.";
    goal_handle->succeed(result);
  }).detach();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto move_group_node = rclcpp::Node::make_shared("move_group_node");
  auto action_node = std::make_shared<SetJointValuesLeftArmWaistServer>(move_group_node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.add_node(action_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
