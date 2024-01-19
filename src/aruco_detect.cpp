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
#include <iostream>
#include <string>
#include <sstream>
#include <thread>

// Gazebo / Ignition
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

// OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"

// ROS2 Msgs
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gz_arm
{

class ArucoDetectNode : public rclcpp::Node
{
public:
  ArucoDetectNode(ignition::transport::Node & ign_node);

protected:
  void clockCb(const ignition::msgs::Clock & msg);

  void cameraInfoCb(const ignition::msgs::CameraInfo & msg);

  void cameraCb(const ignition::msgs::Image & msg);

  void extractPose(const std::vector<cv::Point2f> & rect, rclcpp::Time stamp);

  std::mutex mutex_;
  bool have_camera_info_ = false;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coeffs_;
  std::vector<cv::Point3d> obj_points_;

  // Params
  int target_aruco_id_ = 0;
  double aruco_size_ = 0.1;
  std::string frame_id_ = "camera_link";

  // pub/sub
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
};


std::shared_ptr<rclcpp::Node> createArucoDetectNode(ignition::transport::Node & ign_node)
{
  return std::make_shared<ArucoDetectNode>(ign_node);
}


ArucoDetectNode::ArucoDetectNode(ignition::transport::Node & ign_node)
: rclcpp::Node{"aruco_detect"},
  dictionary_{cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50)},
  parameters_{new cv::aruco::DetectorParameters}
{
  parameters_->minCornerDistanceRate = 0.1;

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
    desc.name = param_ns + "target_aruco_id";
    desc.read_only = true;
    desc.description = "ID of Aurco tag to target";
    desc.integer_range.emplace_back();
    desc.integer_range.at(0).from_value = 0;
    desc.integer_range.at(0).to_value = std::numeric_limits<int>::max();
    target_aruco_id_ = this->declare_parameter(desc.name, 0, desc);
  }

  // Set coordinate system
  obj_points_ = {
    cv::Point3d{-0.5 * aruco_size_, +0.5 * aruco_size_, 0.0},
    cv::Point3d{+0.5 * aruco_size_, +0.5 * aruco_size_, 0.0},
    cv::Point3d{+0.5 * aruco_size_, -0.5 * aruco_size_, 0.0},
    cv::Point3d{-0.5 * aruco_size_, -0.5 * aruco_size_, 0.0}
  };

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ar_pose", 10);

  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);

  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ar_viz", 10);

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera", 10);

  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  std::string cam_topic = "/camera";
  if (!ign_node.Subscribe(cam_topic, &ArucoDetectNode::cameraCb, this)) {
    std::ostringstream ss;
    ss << "Error subscribing to Ignition topic [" << cam_topic << "]";
    throw std::runtime_error(ss.str());
  }

  std::string cam_info_topic = "/camera_info";
  if (!ign_node.Subscribe(cam_info_topic, &ArucoDetectNode::cameraInfoCb, this)) {
    std::ostringstream ss;
    ss << "Error subscribing to Ignition camera info topic [" << cam_info_topic << "]";
    throw std::runtime_error(ss.str());
  }

  std::string clock_topic = "/clock";
  if (!ign_node.Subscribe(clock_topic, &ArucoDetectNode::clockCb, this)) {
    std::ostringstream ss;
    ss << "Error subscribing to Ignition topic [" << clock_topic << "]";
    throw std::runtime_error(ss.str());
  }

}

void ArucoDetectNode::clockCb(const ignition::msgs::Clock & msg)
{
  auto clock_msg = std::make_unique<rosgraph_msgs::msg::Clock>();
  clock_msg->clock.sec = msg.sim().sec();
  clock_msg->clock.nanosec = msg.sim().nsec();
  clock_pub_->publish(std::move(clock_msg));
}

void ArucoDetectNode::cameraInfoCb(const ignition::msgs::CameraInfo & msg)
{
  auto camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>();

  if (msg.intrinsics().k_size() != 9) {
    RCLCPP_WARN_STREAM(get_logger(), "Invalid intrinsics size " << msg.intrinsics().k_size());
    return;
  }

  cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
  for (unsigned rr = 0; rr < 3; ++rr) {
    for (unsigned cc = 0; cc < 3; ++cc) {
      camera_matrix.at<double>(rr, cc) = msg.intrinsics().k(rr * 3 + cc);
    }
  }
  for (int ii = 0; ii < msg.intrinsics().k_size(); ++ii) {
    camera_info->k.at(ii) = msg.intrinsics().k(ii);
  }

  cv::Mat distortion_coeffs = cv::Mat::zeros(msg.distortion().k_size(), 1, CV_64F);
  for (int ii = 0; ii < msg.distortion().k_size(); ++ii) {
    distortion_coeffs.at<double>(ii) = msg.distortion().k(ii);
    camera_info->d.emplace_back(msg.distortion().k(ii));
  }

  if (msg.rectification_matrix_size() != 9) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Invalid rectification size " << msg.rectification_matrix_size());
    return;
  }
  for (int ii = 0; ii < msg.rectification_matrix_size(); ++ii) {
    camera_info->r[ii] = msg.rectification_matrix(ii);
  }

  camera_info->header.stamp.sec = msg.header().stamp().sec();
  camera_info->header.stamp.nanosec = msg.header().stamp().nsec();
  camera_info->header.frame_id = frame_id_;

  camera_info->height = msg.height();
  camera_info->width = msg.width();

  /*
    # Projection/camera matrix
    #     [fx'  0  cx' Tx]
    # P = [ 0  fy' cy' Ty]
    #     [ 0   0   1   0]
  */
  if (msg.projection().p_size() != 12) {
    RCLCPP_WARN_STREAM(get_logger(), "Invalid intrinsics size " << msg.projection().p_size());
    return;
  }
  for (int ii = 0; ii < msg.projection().p_size(); ++ii) {
    camera_info->p[ii] = msg.projection().p(ii);
  }

  // TODO lock
  {
    std::lock_guard lock{mutex_};
    camera_matrix_ = camera_matrix;
    distortion_coeffs_ = distortion_coeffs;
    have_camera_info_ = true;
  }

  camera_info_pub_->publish(std::move(camera_info));
}

void ArucoDetectNode::cameraCb(const ignition::msgs::Image & msg)
{
  bool have_camera_info = false;
  {
    std::lock_guard lock{mutex_};
    have_camera_info = have_camera_info_;
  }

  rclcpp::Time stamp(msg.header().stamp().sec(), msg.header().stamp().nsec());

  RCLCPP_INFO_STREAM(
    this->get_logger(), "Camera Msg "
      << " width " << msg.width()
      << " height " << msg.height()
      << " step " << msg.step()
      << " data.size() " << msg.data().size()
      << " format " << msg.pixel_format_type());

  if (msg.pixel_format_type() != ignition::msgs::PixelFormatType::L_INT8) {
    RCLCPP_WARN_STREAM(get_logger(), "Unsupported image pixel type " << msg.pixel_format_type());
  } else if (!have_camera_info) {
    RCLCPP_WARN_STREAM(get_logger(), "No camera info recieved");
  } else {
    // "borrow" image data
    const int cv_type = CV_8U;
    cv::Mat img(msg.height(), msg.width(), cv_type, const_cast<char *>(msg.data().c_str()),
      msg.step());
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rects, rejected;
    cv::aruco::detectMarkers(img, dictionary_, rects, ids, parameters_, rejected);
    RCLCPP_INFO_STREAM(this->get_logger(), "Detected " << ids.size() << " tags");
    bool found_target_id = false;
    for (unsigned idx = 0; idx < ids.size(); ++idx) {
      int id = ids[idx];
      if (id == target_aruco_id_) {
        found_target_id = true;
        extractPose(rects[idx], stamp);
        break;
      }
    }
    if (!found_target_id) {
      RCLCPP_INFO_STREAM(this->get_logger(), "No tag with id " << target_aruco_id_);
    }
  }

  if (static_cast<bool>(image_pub_)) {
    auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
    img_msg->header.frame_id = frame_id_;
    img_msg->header.stamp = stamp;
    img_msg->height = msg.height();
    img_msg->width = msg.width();
    img_msg->encoding = sensor_msgs::image_encodings::MONO8;
    img_msg->step = msg.step();
    img_msg->data = std::vector<uint8_t>(msg.data().begin(), msg.data().end());
    image_pub_->publish(std::move(img_msg));
  }
}


void ArucoDetectNode::extractPose(const std::vector<cv::Point2f> & rect, rclcpp::Time stamp)
{
  cv::Mat distortion_coeffs;
  cv::Mat camera_matrix;
  {
    std::lock_guard lock{mutex_};
    distortion_coeffs = distortion_coeffs_;
    camera_matrix = camera_matrix_;
  }

  // https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  bool use_extrinsic_guess = false;
  // Calculate pose for marker
  bool result = cv::solvePnP(
    obj_points_, rect, camera_matrix, distortion_coeffs, rvec, tvec,
    use_extrinsic_guess, cv::SOLVEPNP_IPPE_SQUARE);
  if (!result) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "solvePnP returned false");
    return;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Aruco ID " << " rvec " << rvec << " tvec " << tvec);

  auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
  pose_msg->header.frame_id = frame_id_;
  pose_msg->header.stamp = stamp;
  auto & pose = pose_msg->pose;
  pose.position.x = tvec[0];
  pose.position.y = tvec[1];
  pose.position.z = tvec[2];

  // https://answers.ros.org/question/314828/opencv-camera-rvec-tvec-to-ros-world-pose/
  tf2::Quaternion q(tf2::Vector3(rvec[0], rvec[1], rvec[2]), cv::norm(rvec, cv::NORM_L2));
  tf2::convert(q, pose.orientation);

  pose_pub_->publish(std::move(pose_msg));

  auto viz_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
  auto & marker = viz_msg->markers.emplace_back();
  marker.header.frame_id = frame_id_;
  marker.frame_locked = true;
  marker.header.stamp = stamp;
  marker.ns = "aruco_detection";
  marker.id = 0;
  marker.type = marker.CUBE;
  marker.action = marker.ADD;
  marker.pose = pose;
  marker.scale.x = aruco_size_;
  marker.scale.y = aruco_size_;
  marker.scale.z = 0.01; // 1cm
  marker.color.r = 0.0;
  marker.color.g = 0.8;
  marker.color.b = 0.8;
  marker.color.a = 0.5;
  viz_pub_->publish(std::move(viz_msg));
}


}  // namespace gz_arm
