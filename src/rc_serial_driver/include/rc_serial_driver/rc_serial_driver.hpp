// rc_serial_driver/include/rc_serial_driver/rc_serial_driver.hpp

#ifndef RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_
#define RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <mutex>


namespace rc_serial_driver
{

class RCSerialDriver : public rclcpp::Node
{
public:
  explicit RCSerialDriver(const rclcpp::NodeOptions & options);
  ~RCSerialDriver();

private:
  void getParams();
  void initCAN();               // ← 必须声明
  void sendMapBodyPose();       // ← 必须声明
  void reopenPort();
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void yawPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void qualityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  bool readPoseFromTopic(float & x, float & y, float & z, float & yaw);
  bool readYawFromTopic(float & yaw);
  bool readPoseFromTf(float & x, float & y, float & z, float & yaw);
  bool canSendPoseByQuality();

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // CAN
  int can_socket_ = -1;
  std::string can_interface_;
  int can_id_frame1_;  // 用于 x, y
  int can_id_frame2_;  // 用于 z, yaw
  int can_bitrate_;
  std::chrono::steady_clock::time_point last_can_reopen_attempt_{};

  // Pose input
  std::string pose_source_;
  std::string pose_topic_;
  double pose_topic_timeout_sec_{0.5};
  std::mutex pose_mutex_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  bool yaw_pose_topic_enabled_{false};
  bool yaw_pose_topic_strict_{false};
  std::string yaw_pose_topic_;
  double yaw_pose_topic_timeout_sec_{0.5};
  std::mutex yaw_pose_mutex_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_yaw_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr yaw_pose_sub_;

  // Relocalization quality gate
  bool quality_gate_enabled_{false};
  std::string quality_topic_;
  double quality_timeout_sec_{1.0};
  double quality_max_fitness_{0.05};
  int quality_min_consecutive_accepts_{5};
  bool quality_require_accepted_{true};
  bool quality_require_converged_{true};
  std::mutex quality_mutex_;
  std_msgs::msg::Float32MultiArray::SharedPtr latest_quality_;
  rclcpp::Time latest_quality_wall_time_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr quality_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr send_timer_;  // ← 必须声明
};

}  // namespace rc_serial_driver

#endif  // RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_
