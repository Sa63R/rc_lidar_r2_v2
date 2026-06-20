// rc_serial_driver/include/rc_serial_driver/rc_serial_driver.hpp

#ifndef RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_
#define RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
  bool readPoseFromTopic(float & x, float & y, float & z, float & yaw);
  bool readPoseFromTf(float & x, float & y, float & z, float & yaw);

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

  // Timer
  rclcpp::TimerBase::SharedPtr send_timer_;  // ← 必须声明
};

}  // namespace rc_serial_driver

#endif  // RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_
