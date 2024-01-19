/*
Copyright (c) 2024 Derek King

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Standard Libaray
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

// ROS2
#include "rclcpp/rclcpp.hpp"

// ROS2 Msgs
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>

// GZ Arm
#include "gz_arm/gz_arm_util.hpp"
#include "gz_arm/gz_arm_control.hpp"

namespace gz_arm
{

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class GzArmControl : public rclcpp::Node
{
public:
  GzArmControl();

protected:
  void jointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr & msg);

  void arPoseCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);

  void twistCmdCb(const geometry_msgs::msg::Twist::ConstSharedPtr & msg);

  void timerCb();

  void pubViz(
    double global_ar_x, double global_ar_y, double global_ar_z,
    const tf2::Quaternion & global_ar_q, double target_x, double target_y,
    rclcpp::Time stamp);

  std::tuple<double, double, double> calcEffectorVelocities(rclcpp::Time stamp);

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  sensor_msgs::msg::JointState::ConstSharedPtr joint_states_;
  geometry_msgs::msg::Twist::ConstSharedPtr twist_cmd_;
  rclcpp::Time twist_cmd_stamp_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr ar_pose_;

  // Params
  double aruco_size_ = 0.1;
  double target_offset_ = 0.4;
  double angular_gain_ = 0.1;
  double positional_gain_ = 0.1;
  double linear_velocity_limit_ = 0.05;
  double joint_velocity_limit_ = 0.5;

  // pub/sub
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ar_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
};

std::shared_ptr<rclcpp::Node> createGzArmControlNode()
{
  return std::make_shared<GzArmControl>();
}

GzArmControl::GzArmControl()
: Node("gz_arm_control")
{
  std::string param_ns{get_name()};
  param_ns.push_back('.');

  {
    rcl_interfaces::msg::ParameterDescriptor desc{};
    desc.name = param_ns + "aruco_size";
    desc.read_only = true;
    desc.description = "edge length of aruco marker (for visualization purposes)";
    desc.floating_point_range.emplace_back();
    desc.floating_point_range.at(0).from_value = 0.0;
    desc.floating_point_range.at(0).to_value = std::numeric_limits<double>::infinity();
    aruco_size_ = this->declare_parameter(desc.name, 0.1, desc);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor desc{};
    desc.name = param_ns + "target_offset";
    desc.read_only = true;
    desc.description = "offset distance from aruco tag to target for arm end-effector";
    desc.floating_point_range.emplace_back();
    desc.floating_point_range.at(0).from_value = 0.0;
    desc.floating_point_range.at(0).to_value = std::numeric_limits<double>::infinity();
    target_offset_ = this->declare_parameter(desc.name, 0.4, desc);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor desc{};
    desc.name = param_ns + "angular_gain";
    desc.read_only = true;
    desc.description = "Control gain on end-effector angle (to target)";
    desc.floating_point_range.emplace_back();
    desc.floating_point_range.at(0).from_value = 0.0;
    desc.floating_point_range.at(0).to_value = std::numeric_limits<double>::infinity();
    angular_gain_ = this->declare_parameter(desc.name, 1.0, desc);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor desc{};
    desc.name = param_ns + "positional_gain";
    desc.read_only = true;
    desc.description = "Control gain on end-effector x,y offset (to target)";
    desc.floating_point_range.emplace_back();
    desc.floating_point_range.at(0).from_value = 0.0;
    desc.floating_point_range.at(0).to_value = std::numeric_limits<double>::infinity();
    positional_gain_ = this->declare_parameter(desc.name, 0.1, desc);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor desc{};
    desc.name = param_ns + "linear_velocity_limit";
    desc.read_only = true;
    desc.description = "Limit on end-effector linear velocity";
    desc.floating_point_range.emplace_back();
    desc.floating_point_range.at(0).from_value = 0.0;
    desc.floating_point_range.at(0).to_value = std::numeric_limits<double>::infinity();
    linear_velocity_limit_ = this->declare_parameter(desc.name, 0.05, desc);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor desc{};
    desc.name = param_ns + "joint_velocity_limit";
    desc.read_only = true;
    desc.description = "Limit on individual joint velocities";
    desc.floating_point_range.emplace_back();
    desc.floating_point_range.at(0).from_value = 0.0;
    desc.floating_point_range.at(0).to_value = std::numeric_limits<double>::infinity();
    joint_velocity_limit_ = this->declare_parameter(desc.name, 0.05, desc);
  }

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cmd_viz", 10);

  cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/velocity_controller/commands", 10);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&GzArmControl::jointStateCb, this, _1),
    sub_options);

  timer_ = this->create_wall_timer(
    500ms, std::bind(&GzArmControl::timerCb, this), callback_group_);

  ar_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ar_pose", 10, std::bind(&GzArmControl::arPoseCb, this, _1),
    sub_options);

  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&GzArmControl::twistCmdCb, this, _1),
    sub_options);
}

void GzArmControl::jointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
{
  joint_states_ = msg;
}

void GzArmControl::arPoseCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  ar_pose_ = msg;
}

void GzArmControl::twistCmdCb(const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "twist cmd msg");
  rclcpp::Time stamp = this->get_clock()->now();
  twist_cmd_ = msg;
  twist_cmd_stamp_ = stamp;
}


std::tuple<double, double, double> GzArmControl::calcEffectorVelocities(rclcpp::Time stamp)
{
  double vx = 0.0, vy = 0.0, vth = 0.0;
  auto [cam_x, cam_y, cam_th] = camFromJoints(joint_states_->position);
  double cam_z = 0.275; // TODO (get from cam from joints)

  // ar_tag in global frame
  const double cam_c = std::cos(cam_th);
  const double cam_s = std::sin(cam_th);
  const double ar_x = ar_pose_->pose.position.z;  // link3 x-axis = cam z-axis
  const double ar_y = -ar_pose_->pose.position.x;  // link3 y-axis = neg cam x-axis
  const double ar_z = -ar_pose_->pose.position.y;  // link3 z-axis = neg cam y-axis
  const double global_ar_x = cam_x + cam_c * ar_x - cam_s * ar_y;
  const double global_ar_y = cam_y + cam_s * ar_x + cam_c * ar_y;
  const double global_ar_z = cam_z + ar_z;

  // convert cam_th into quaternion
  // x,y,z,w
  tf2::Quaternion cam_q(0, 0, std::sin(0.5 * cam_th), std::cos(0.5 * cam_th));
  tf2::Quaternion ar_q(ar_pose_->pose.orientation.z,
    -ar_pose_->pose.orientation.x,
    -ar_pose_->pose.orientation.y,
    ar_pose_->pose.orientation.w);
  tf2::Quaternion global_ar_q = cam_q * ar_q;

  const double global_ar_th = angleAroundZAxis(global_ar_q);

  const double target_x = global_ar_x + std::cos(global_ar_th) * target_offset_;
  const double target_y = global_ar_y + std::sin(global_ar_th) * target_offset_;

  this->pubViz(global_ar_x, global_ar_y, global_ar_z, global_ar_q, target_x, target_y, stamp);

  RCLCPP_INFO_STREAM(this->get_logger(), "ar_pose " << ar_x << ',' << ar_y << ',' << ar_z);
  RCLCPP_INFO_STREAM(
    this->get_logger(), "cam_pose " << cam_x << ',' << cam_y << ',' << cam_z << ',' << cam_th);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "ar_global_pose " << global_ar_x << ',' << global_ar_y << ',' << global_ar_z);

  // compute angle from end-effector to center of AR marker
  // want to angle end-effector so it doesn't loose sight of marker
  const double ar_heading = std::atan2(ar_y, ar_x);
  // try to have velocity push ar heading to zero
  vth = angular_gain_ * ar_heading;   // use gain of 1.0 for now

  vx = positional_gain_ * (target_x - cam_x);
  vy = positional_gain_ * (target_y - cam_y);
  std::clamp(vx, -linear_velocity_limit_, linear_velocity_limit_);
  std::clamp(vy, -linear_velocity_limit_, linear_velocity_limit_);
  RCLCPP_INFO_STREAM(this->get_logger(), "cmd " << vx << ',' << vy << ',' << vth);

  return std::make_tuple(vx, vy, vth);
}


void GzArmControl::timerCb()
{
  rclcpp::Time stamp = this->get_clock()->now();

  //double nan = std::numeric_limits::quietNaN();
  auto cmd_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  cmd_msg->data = {0.0, 0.0, 0.0};

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  const rclcpp::Duration ar_timeout(0, 200 * 1000 * 1000);
  const rclcpp::Duration cmd_timeout(1, 0);

  if (!joint_states_) {
    RCLCPP_WARN(this->get_logger(), "joint states has not been received yet");
  } else if ((joint_states_->name.size() != 3) or (joint_states_->position.size() != 3)) {
    RCLCPP_ERROR(this->get_logger(), "joint states names and/or position are not size 3");
  } else if ((joint_states_->name[0] != "base_to_arm1_joint") or
    (joint_states_->name[1] != "arm1_to_arm2_joint") or
    (joint_states_->name[2] != "arm2_to_arm3_joint"))
  {
    RCLCPP_ERROR(this->get_logger(), "incorrect joint state names");
  } else {
    if (!ar_pose_) {
      RCLCPP_WARN(this->get_logger(), "no ar_pose received yet");
    } else if (auto ar_age = (stamp - ar_pose_->header.stamp); ar_age > ar_timeout) {
      RCLCPP_WARN_STREAM(this->get_logger(), "ar_pose timeout : age " << ar_age.seconds());
    } else {
      // compute target pose in global coordinate system
      std::tie(vx, vy, vth) = calcEffectorVelocities(stamp);
    }

    if (!twist_cmd_) {
      RCLCPP_WARN(this->get_logger(), "no twist_cmd received yet");
    } else if (auto cmd_age = (stamp - twist_cmd_stamp_); cmd_age > cmd_timeout) {
      RCLCPP_WARN_STREAM(this->get_logger(), "twist_cmd timeout : age " << cmd_age.seconds());
    } else {
      vx = twist_cmd_->linear.x;
      vy = twist_cmd_->linear.y;
      vth = twist_cmd_->angular.z;
    }

    cmd_msg->data = endEffectorToJointVelocities(
      vx, vy, vth, joint_states_->position,
      joint_velocity_limit_);
    for (double v : cmd_msg->data) {
      if (!std::isfinite(v)) {
        std::ostringstream ss;
        ss << "Command contains non-finite values ";
        for (double v : cmd_msg->data) {
          ss << v << ' ';
        }
        ss << " vx " << vx
           << " vy " << vy
           << " vth " << vth;
        RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
        std::fill(cmd_msg->data.begin(), cmd_msg->data.end(), 0.0);
        break;
      }
    }
  }

  cmd_pub_->publish(std::move(cmd_msg));
}


void GzArmControl::pubViz(
  double global_ar_x, double global_ar_y, double global_ar_z,
  const tf2::Quaternion & global_ar_q,
  double target_x, double target_y, rclcpp::Time stamp)
{
  auto viz_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
  {
    auto & marker = viz_msg->markers.emplace_back();
    marker.header.frame_id = "base_link";
    marker.frame_locked = true;
    marker.header.stamp = stamp;
    marker.ns = "ef_target";
    marker.id = 0;
    marker.type = marker.SPHERE;
    marker.action = marker.ADD;
    marker.pose.position.x = target_x;
    marker.pose.position.y = target_y;
    marker.pose.position.z = global_ar_z;
    // todo is there as toMsg or convert from a quaternion to a rotation
    tf2::convert(global_ar_q, marker.pose.orientation);
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
  }

  {
    auto & marker = viz_msg->markers.emplace_back();
    marker.header.frame_id = "base_link";
    marker.frame_locked = true;
    marker.header.stamp = stamp;
    marker.ns = "ef_target";
    marker.id = 1;
    marker.type = marker.ARROW;
    marker.action = marker.ADD;
    marker.points.resize(2);
    marker.points[0].x = global_ar_x;
    marker.points[0].y = global_ar_y;
    marker.points[0].z = global_ar_z;
    marker.points[1].x = target_x;
    marker.points[1].y = target_y;
    marker.points[1].z = global_ar_z;
    marker.scale.x = 0.02;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
  }

  {
    auto & marker = viz_msg->markers.emplace_back();
    marker.header.frame_id = "base_link";
    marker.frame_locked = true;
    marker.header.stamp = stamp;
    marker.ns = "aruco_detection";
    marker.id = 0;
    marker.type = marker.CUBE;
    marker.action = marker.ADD;
    marker.pose.position.x = global_ar_x;
    marker.pose.position.y = global_ar_y;
    marker.pose.position.z = global_ar_z;
    // todo is there as toMsg or convert from a quaternion to a rotation
    tf2::convert(global_ar_q, marker.pose.orientation);
    marker.scale.x = 0.01; // 1cm
    marker.scale.y = aruco_size_;
    marker.scale.z = aruco_size_;
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
  }
  viz_pub_->publish(std::move(viz_msg));
}


}  // namespace gz_arm
