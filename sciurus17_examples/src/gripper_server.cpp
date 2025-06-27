// gripper_server.cpp — Dual-instance‑safe version
// Each side runs in its **own namespace**, so two terminals can run simultaneously:
//   ▸ ros2 run sciurus17_examples gripper_server right   # → /right namespace
//   ▸ ros2 run sciurus17_examples gripper_server left    # → /left  namespace
// Services become:
//   /right/set_hand_angle, /left/set_hand_angle … など
// -------------------------------------------------------
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <iostream>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "rclcpp/rclcpp.hpp"

#include "sciurus17_gripper_interfaces/srv/set_hand_angle.hpp"
#include "sciurus17_gripper_interfaces/srv/get_hand_angle.hpp"

using moveit::planning_interface::MoveGroupInterface;
using SetHandAngle = sciurus17_gripper_interfaces::srv::SetHandAngle;
using GetHandAngle = sciurus17_gripper_interfaces::srv::GetHandAngle;

class GripperServer : public rclcpp::Node
{
public:
  GripperServer(const std::string & ns, const rclcpp::NodeOptions & options)
    : Node("gripper_server", ns, options)
  {
    // Declare only if not already overridden.
    if (!has_parameter("gripper_group")) {
      declare_parameter("gripper_group", "r_gripper_group");
    }
    if (!has_parameter("open_limit_deg")) {
      declare_parameter("open_limit_deg", 40.0);
    }
    declare_parameter("velocity_scaling", 1.0);
    declare_parameter("acceleration_scaling", 1.0);

    gripper_group_name_ = get_parameter("gripper_group").as_string();
    const double vel = get_parameter("velocity_scaling").as_double();
    const double acc = get_parameter("acceleration_scaling").as_double();

    move_group_node_ = rclcpp::Node::make_shared("gripper_move_group_node");
    executor_.add_node(move_group_node_);
    executor_thread_ = std::thread([this] { executor_.spin(); });

    move_group_ = std::make_unique<MoveGroupInterface>(move_group_node_, gripper_group_name_);
    move_group_->setMaxVelocityScalingFactor(vel);
    move_group_->setMaxAccelerationScalingFactor(acc);

    RCLCPP_INFO(get_logger(), "[%s] ready for group '%s'", ns.c_str(), gripper_group_name_.c_str());

    set_srv_ = create_service<SetHandAngle>(
    "set_hand_angle",
    std::bind(&GripperServer::handle_set_hand_angle, this,
                std::placeholders::_1, std::placeholders::_2));

    get_srv_ = create_service<GetHandAngle>(
    "get_hand_angle",
    std::bind(&GripperServer::handle_get_hand_angle, this,
                std::placeholders::_1, std::placeholders::_2));
  }

  ~GripperServer() override {
    executor_.cancel(); if (executor_thread_.joinable()) executor_thread_.join();
  }

private:
  double signed_limit_deg() const {
    const double lim = get_parameter("open_limit_deg").as_double();
    return (gripper_group_name_.rfind("l_",0)==0) ? -std::abs(lim) : std::abs(lim);
  }

  void handle_set_hand_angle(const std::shared_ptr<SetHandAngle::Request> req,
                             std::shared_ptr<SetHandAngle::Response> res) {
    const double user=req->angle;
    const double lim=signed_limit_deg();
    const double tgt_deg=(0<=user && user<=100)?user/100.0*lim:user;
    const double tgt_rad=angles::from_degrees(tgt_deg);
    try{
      auto j=move_group_->getCurrentJointValues(); if(j.empty()) throw std::runtime_error("no joints");
      j[0]=tgt_rad; move_group_->setJointValueTarget(j);
      auto ok=move_group_->move();
      res->success=(ok==moveit::core::MoveItErrorCode::SUCCESS);
      res->message=res->success?"Moved":"MoveIt error "+std::to_string(ok.val);
    }catch(const std::exception&e){res->success=false; res->message=e.what();}
  }

  void handle_get_hand_angle(const std::shared_ptr<GetHandAngle::Request>, std::shared_ptr<GetHandAngle::Response> res){
    try{auto j=move_group_->getCurrentJointValues(); if(j.empty()) throw std::runtime_error("no joints"); res->angle=static_cast<int32_t>(std::lround(angles::to_degrees(j[0])));}catch(...){res->angle=std::numeric_limits<int32_t>::min();}
  }

  std::string gripper_group_name_;
  std::unique_ptr<MoveGroupInterface> move_group_;
  rclcpp::Service<SetHandAngle>::SharedPtr set_srv_;
  rclcpp::Service<GetHandAngle>::SharedPtr get_srv_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string group="l_gripper_group", ns="/left"; double limit=-40.0;
  if(argc>1){std::string s=argv[1]; if(s=="right"){group="r_gripper_group"; ns="/right"; limit=40.0;} else if(s!="left"){std::cerr<<"Usage: gripper_server [left|right]"<<std::endl; return 1;}}
  auto opts=rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
              .append_parameter_override("gripper_group",group)
              .append_parameter_override("open_limit_deg",limit);
  rclcpp::spin(std::make_shared<GripperServer>(ns, opts));
  rclcpp::shutdown(); return 0;
}
