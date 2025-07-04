// gripper_torque_server.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "sciurus17_gripper_interfaces/srv/set_hand_torque.hpp"

using SetHandTorque = sciurus17_gripper_interfaces::srv::SetHandTorque;

class GripperTorqueServer : public rclcpp::Node
{
public:
  GripperTorqueServer(const std::string & ns,
                      const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
    : Node("gripper_torque_server", ns, opts)
  {
    /* ---------------- パラメータ ---------------- */
    declare_parameter("effort_topic", "hand_effort_controller/command");  // 名前空間内
    declare_parameter("effort_max",   0.4);   // N·m – HWリミット
    declare_parameter("effort_min",  -0.4);   // N·m

    /* ------------- 発行トピック準備 ------------- */
    const std::string topic = get_parameter("effort_topic").as_string();
    effort_pub_ = create_publisher<std_msgs::msg::Float64>(topic, 1);

    /* ------------- サービス準備 ------------- */
    srv_ = create_service<SetHandTorque>(
      "set_hand_torque",
      std::bind(&GripperTorqueServer::handle_set_hand_torque,
                this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "[%s] ready, publishing -> %s",
                ns.c_str(), topic.c_str());
  }

private:
  void handle_set_hand_torque(const std::shared_ptr<SetHandTorque::Request> req,
                              std::shared_ptr<SetHandTorque::Response> res)
  {
    const double eff_max = get_parameter("effort_max").as_double();
    const double eff_min = get_parameter("effort_min").as_double();
    const double cmd     = std::clamp(req->torque, eff_min, eff_max);

    std_msgs::msg::Float64 msg;
    msg.data = cmd;
    effort_pub_->publish(msg);

    res->success = true;
    res->message = "Effort command sent: " + std::to_string(cmd) + " N·m";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr effort_pub_;
  rclcpp::Service<SetHandTorque>::SharedPtr             srv_;
};

/* --------------------- main --------------------- */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string ns  = "/left";   // デフォルト
  double sign     = -1.0;      // 左手は「閉じる＝負 Effort」
  if (argc > 1) {
    std::string s(argv[1]);
    if (s == "right") { ns = "/right"; sign =  1.0; }
    else if (s != "left") {
      std::cerr << "Usage: gripper_torque_server [left|right]\n";
      return 1;
    }
  }

  auto opts = rclcpp::NodeOptions()
                .append_parameter_override("effort_max",  0.4 * sign)
                .append_parameter_override("effort_min", -0.4 * sign);

  rclcpp::spin(std::make_shared<GripperTorqueServer>(ns, opts));
  rclcpp::shutdown();
  return 0;
}
