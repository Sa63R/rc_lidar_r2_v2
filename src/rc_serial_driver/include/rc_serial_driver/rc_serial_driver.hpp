// rc_serial_driver/include/rc_serial_driver/rc_serial_driver.hpp

#ifndef RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_
#define RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>



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
  

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // CAN
  int can_socket_ = -1;
  std::string can_interface_;
  int can_id_frame1_;  // 用于 x, y
  int can_id_frame2_;  // 用于 z, yaw
  int can_bitrate_;

  // Timer
  rclcpp::TimerBase::SharedPtr send_timer_;  // ← 必须声明
};

}  // namespace rc_serial_driver

#endif  // RC_SERIAL_DRIVER__RC_SERIAL_DRIVER_HPP_