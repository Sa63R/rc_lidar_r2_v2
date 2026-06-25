// Copyright (c) 2026 Zhong Wenjie
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system headers
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/if.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rc_serial_driver/rc_serial_driver.hpp"

#include <linux/can/raw.h>
#include <unistd.h>
#include <sys/types.h>

namespace rc_serial_driver
{

namespace
{
constexpr uint8_t RESTART_PAYLOAD[8] = {0xA5, 0x5A, 0x01, 0xFE, 0x00, 0x00, 0x00, 0x00};

int g_can_id_restart = 0x30;
std::string g_restart_script = "/home/orangepi/RC_LIDAR_NOW/autostart/restart_lidar_stack.sh";
bool g_restart_enabled = true;
std::chrono::steady_clock::time_point g_last_restart_time{};
rclcpp::TimerBase::SharedPtr g_recv_timer;

void applyRestartCanFilter(int can_socket, int can_id_restart, const rclcpp::Logger & logger)
{
  struct can_filter rfilter[1];
  rfilter[0].can_id = static_cast<canid_t>(can_id_restart);
  rfilter[0].can_mask = CAN_SFF_MASK;

  if (setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
    RCLCPP_WARN(logger, "Failed to set CAN RX filter: %s", strerror(errno));
  }
}

bool isRestartFrame(const struct can_frame & frame)
{
  const canid_t rx_id = frame.can_id & CAN_SFF_MASK;

  return rx_id == static_cast<canid_t>(g_can_id_restart) &&
    frame.can_dlc == 8 &&
    std::memcmp(frame.data, RESTART_PAYLOAD, sizeof(RESTART_PAYLOAD)) == 0;
}

bool isCanTxQueueFullError(int error_number)
{
  switch (error_number) {
    case EAGAIN:
    case ENOBUFS:
#if EWOULDBLOCK != EAGAIN
    case EWOULDBLOCK:
#endif
      return true;
    default:
      return false;
  }
}

float yawFromQuaternion(const geometry_msgs::msg::Quaternion & q_msg)
{
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  tf2::Matrix3x3 m(q);
  double roll;
  double pitch;
  double yaw;
  m.getRPY(roll, pitch, yaw);
  return static_cast<float>(yaw);
}

void requestRadarRestart(const rclcpp::Logger & logger)
{
  const auto now = std::chrono::steady_clock::now();

  if (g_last_restart_time.time_since_epoch().count() != 0 &&
    now - g_last_restart_time < std::chrono::seconds(8))
  {
    RCLCPP_WARN(logger, "Restart command ignored because of debounce");
    return;
  }

  g_last_restart_time = now;

  RCLCPP_WARN(
    logger,
    "Received CAN restart command. Running restart script: %s",
    g_restart_script.c_str());

  pid_t pid = fork();

  if (pid < 0) {
    RCLCPP_ERROR(logger, "fork() failed: %s", strerror(errno));
    return;
  }

  if (pid == 0) {
    setsid();
    execl("/bin/bash", "bash", g_restart_script.c_str(), static_cast<char *>(nullptr));
    _exit(127);
  }

  RCLCPP_WARN(logger, "Restart script launched in background, pid=%d", pid);
}
}  // namespace

RCSerialDriver::RCSerialDriver(const rclcpp::NodeOptions & options)
: Node("rc_serial_driver", options)
{
  RCLCPP_INFO(get_logger(), "Starting RCSerialDriver (send-only map→body pose via CAN)...");

  getParams();
  

  // TF buffer and listener for map->body transform
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_,
    10,
    std::bind(&RCSerialDriver::poseCallback, this, std::placeholders::_1));
  if (yaw_pose_topic_enabled_) {
    yaw_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      yaw_pose_topic_,
      20,
      std::bind(&RCSerialDriver::yawPoseCallback, this, std::placeholders::_1));
  }

  if (quality_gate_enabled_) {
    quality_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      quality_topic_,
      10,
      std::bind(&RCSerialDriver::qualityCallback, this, std::placeholders::_1));
  }

  // Timer to send pose at 50 Hz
  send_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), //发送周期 20ms,频率为50Hz
    [this]() { this->sendMapBodyPose(); });

  // Initialize CAN socket (assume interface is pre-configured)
  initCAN();

  // Timer to receive restart command at 100 Hz
  g_recv_timer = this->create_wall_timer(
    std::chrono::milliseconds(10),
    [this]() {
      if (!g_restart_enabled || can_socket_ < 0) {
        return;
      }

      while (true) {
        struct can_frame frame {};
        ssize_t n = read(can_socket_, &frame, sizeof(frame));

        if (n < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
          }

          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "CAN receive error: %s", strerror(errno));
          reopenPort();
          break;
        }

        if (n != sizeof(struct can_frame)) {
          continue;
        }

        const canid_t rx_id = frame.can_id & CAN_SFF_MASK;
        if (rx_id != static_cast<canid_t>(g_can_id_restart)) {
          continue;
        }

        if (isRestartFrame(frame)) {
          requestRadarRestart(get_logger());
        } else {
          RCLCPP_WARN(
            get_logger(),
            "Received restart CAN ID 0x%X but payload mismatch",
            g_can_id_restart);
        }
      }
    });
}

RCSerialDriver::~RCSerialDriver()
{
  g_recv_timer.reset();

  if (can_socket_ >= 0) {
    close(can_socket_);
  }
}

void RCSerialDriver::getParams()
{
  can_interface_ = declare_parameter<std::string>("can_interface", "can0");
  can_id_frame1_ = declare_parameter<int>("can_id_frame1", 0x12);
  can_id_frame2_ = declare_parameter<int>("can_id_frame2", 0x13);
  can_bitrate_ = declare_parameter<int>("can_bitrate", 1000000);
  pose_source_ = declare_parameter<std::string>("pose_source", "tf");
  pose_topic_ = declare_parameter<std::string>("pose_topic", "/lidar/pose");
  pose_topic_timeout_sec_ = declare_parameter<double>("pose_topic_timeout_sec", 0.5);
  yaw_pose_topic_enabled_ = declare_parameter<bool>("yaw_pose_topic_enabled", false);
  yaw_pose_topic_ = declare_parameter<std::string>("yaw_pose_topic", "/robot/pose");
  yaw_pose_topic_timeout_sec_ = declare_parameter<double>("yaw_pose_topic_timeout_sec", 0.5);
  yaw_pose_topic_strict_ = declare_parameter<bool>("yaw_pose_topic_strict", false);
  yaw_ignore_quality_gate_ = declare_parameter<bool>("yaw_ignore_quality_gate", true);
  quality_gate_enabled_ = declare_parameter<bool>("quality_gate_enabled", false);
  quality_topic_ = declare_parameter<std::string>("quality_topic", "/map_lidar/quality");
  quality_timeout_sec_ = declare_parameter<double>("quality_timeout_sec", 1.0);
  quality_max_fitness_ = declare_parameter<double>("quality_max_fitness", 0.05);
  quality_min_consecutive_accepts_ = declare_parameter<int>("quality_min_consecutive_accepts", 5);
  quality_require_accepted_ = declare_parameter<bool>("quality_require_accepted", true);
  quality_require_converged_ = declare_parameter<bool>("quality_require_converged", true);

  if (pose_source_ != "tf" && pose_source_ != "pose_topic" && pose_source_ != "auto") {
    RCLCPP_WARN(
      get_logger(),
      "Unknown pose_source '%s', falling back to 'tf'. Expected tf, pose_topic, or auto.",
      pose_source_.c_str());
    pose_source_ = "tf";
  }
  
  g_can_id_restart = declare_parameter<int>("can_id_restart", 0x30);
  g_restart_script = declare_parameter<std::string>(
    "restart_script",
    "/home/orangepi/RC_LIDAR_NOW/autostart/restart_lidar_stack.sh");
  g_restart_enabled = declare_parameter<bool>("restart_enabled", true);

  RCLCPP_INFO(get_logger(), "Restart CAN ID: 0x%X", g_can_id_restart);
  RCLCPP_INFO(get_logger(), "Restart script: %s", g_restart_script.c_str());

  RCLCPP_INFO(get_logger(), "CAN Interface: %s", can_interface_.c_str());
  RCLCPP_INFO(get_logger(), "Expected CAN Bitrate: %d bit/s (ensure pre-configured)", can_bitrate_);
  RCLCPP_INFO(
    get_logger(),
    "Pose source: %s (pose_topic=%s timeout=%.2fs)",
    pose_source_.c_str(),
    pose_topic_.c_str(),
    pose_topic_timeout_sec_);
  RCLCPP_INFO(
    get_logger(),
    "CAN yaw topic override: %s (topic=%s timeout=%.2fs strict=%s ignore_quality_gate=%s)",
    yaw_pose_topic_enabled_ ? "enabled" : "disabled",
    yaw_pose_topic_.c_str(),
    yaw_pose_topic_timeout_sec_,
    yaw_pose_topic_strict_ ? "true" : "false",
    yaw_ignore_quality_gate_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(),
    "Relocalization quality gate: %s (topic=%s timeout=%.2fs max_fitness=%.4f min_accepts=%d)",
    quality_gate_enabled_ ? "enabled" : "disabled",
    quality_topic_.c_str(),
    quality_timeout_sec_,
    quality_max_fitness_,
    quality_min_consecutive_accepts_);
}

void RCSerialDriver::initCAN()
{
  try {
    // Open CAN raw socket
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      throw std::runtime_error("Failed to create CAN socket: " + std::string(strerror(errno)));
    }

    // Set non-blocking mode
    fcntl(can_socket_, F_SETFL, O_NONBLOCK);

    // Increase send buffer size (1 MB)
    int send_buf_size = 1 << 20;
    setsockopt(can_socket_, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));

    // Get interface index
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      close(can_socket_);
      throw std::runtime_error("Failed to get interface index for '" + can_interface_ + "'");
    }

    // Bind to CAN interface
    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      close(can_socket_);
      throw std::runtime_error("Failed to bind CAN socket to '" + can_interface_ + "'");
    }

    applyRestartCanFilter(can_socket_, g_can_id_restart, get_logger());

    RCLCPP_INFO(get_logger(), "CAN initialized on %s", can_interface_.c_str());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "CAN initialization failed: %s", ex.what());
    can_socket_ = -1;
  }
}

void RCSerialDriver::sendMapBodyPose()
{
  if (can_socket_ < 0) {
    const auto now = std::chrono::steady_clock::now();
    if (last_can_reopen_attempt_.time_since_epoch().count() == 0 ||
      now - last_can_reopen_attempt_ >= std::chrono::seconds(2))
    {
      last_can_reopen_attempt_ = now;
      RCLCPP_WARN(get_logger(), "CAN socket not ready, trying to reopen.");
      reopenPort();
    }

    if (can_socket_ < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "CAN socket not ready, skipping send.");
      return;
    }
  }

  try {
    float x_can = 0.0f;
    float y_can = 0.0f;
    float z_can = 0.0f;
    float yaw_can = 0.0f;
    std::string source_used;

    if (pose_source_ == "pose_topic" || pose_source_ == "auto") {
      if (readPoseFromTopic(x_can, y_can, z_can, yaw_can)) {
        source_used = pose_topic_;
      }
    }

    if (source_used.empty() && pose_source_ != "pose_topic") {
      if (readPoseFromTf(x_can, y_can, z_can, yaw_can)) {
        source_used = "TF robot_origin->robot_base_highrate";
      }
    }

    if (source_used.empty()) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Pose source not ready: %s",
        pose_source_.c_str());
      return;
    }

    std::string yaw_source_used;
    if (yaw_pose_topic_enabled_) {
      float fused_yaw = 0.0f;
      if (readYawFromTopic(fused_yaw)) {
        yaw_can = fused_yaw;
        yaw_source_used = yaw_pose_topic_;
      } else if (yaw_pose_topic_strict_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "CAN yaw topic %s is not ready; holding CAN pose output.",
          yaw_pose_topic_.c_str());
        return;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "CAN yaw topic %s is not ready; using yaw from %s.",
          yaw_pose_topic_.c_str(),
          source_used.c_str());
      }
    }

    const bool quality_ready = canSendPoseByQuality();
    const bool yaw_can_bypass_quality =
      !quality_ready &&
      yaw_pose_topic_enabled_ &&
      yaw_ignore_quality_gate_ &&
      !yaw_source_used.empty();

    if (!quality_ready && !yaw_can_bypass_quality) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Relocalization quality is not ready; holding CAN pose output.");
      return;
    }

    if (quality_ready) {
      has_quality_passed_z_can_ = true;
      last_quality_passed_z_can_ = z_can;
    } else {
      if (!has_quality_passed_z_can_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Relocalization quality is not ready and no trusted z is available; holding CAN yaw output.");
        return;
      }

      z_can = last_quality_passed_z_can_;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Relocalization quality is not ready; holding CAN x/y but sending yaw from %s with last trusted z.",
        yaw_source_used.c_str());
    }

    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "Sending pose from " << source_used
      << (yaw_source_used.empty() ? "" : (" yaw_from=" + yaw_source_used))
      << ": can_output[x_right=" << x_can
      << " y_forward=" << y_can
      << " z_up=" << z_can
      << " yaw_left_positive=" << yaw_can
      << "]");

    if (quality_ready) {
      // === 帧1: x_can, y_can (ID = 0x12) ===
      struct can_frame frame1;
      frame1.can_id = can_id_frame1_;
      frame1.can_dlc = 8;
      std::memcpy(frame1.data + 0, &x_can, sizeof(float));
      std::memcpy(frame1.data + 4, &y_can, sizeof(float));

      ssize_t bytes_written = write(can_socket_, &frame1, sizeof(struct can_frame));
      if (bytes_written < 0) {
        if (isCanTxQueueFullError(errno)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "CAN buffer full, dropping frame1");
        } else {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to send frame1: %s", strerror(errno));
          reopenPort();
          return;
        }
      }
    }

    // === 帧2: z_can, yaw_can (ID = 0x13) ===
    struct can_frame frame2;
    frame2.can_id = can_id_frame2_;
    frame2.can_dlc = 8;
    std::memcpy(frame2.data + 0, &z_can, sizeof(float));
    std::memcpy(frame2.data + 4, &yaw_can, sizeof(float));

    ssize_t bytes_written = write(can_socket_, &frame2, sizeof(struct can_frame));
    if (bytes_written < 0) {
      if (isCanTxQueueFullError(errno)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "CAN buffer full, dropping frame2");
      } else {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to send frame2: %s", strerror(errno));
        reopenPort();
        return;
      }
    }

  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "TF not ready: %s", ex.what());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error in sendMapBodyPose: %s", ex.what());
    reopenPort();
  }
}

void RCSerialDriver::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  latest_pose_ = msg;
}

void RCSerialDriver::yawPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(yaw_pose_mutex_);
  latest_yaw_pose_ = msg;
}

void RCSerialDriver::qualityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(quality_mutex_);
  latest_quality_ = msg;
  latest_quality_wall_time_ = now();
}

bool RCSerialDriver::canSendPoseByQuality()
{
  if (!quality_gate_enabled_) {
    return true;
  }

  std_msgs::msg::Float32MultiArray::SharedPtr quality_msg;
  rclcpp::Time quality_wall_time;
  {
    std::lock_guard<std::mutex> lock(quality_mutex_);
    quality_msg = latest_quality_;
    quality_wall_time = latest_quality_wall_time_;
  }

  if (!quality_msg || quality_msg->data.size() < 10) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Waiting for relocalization quality topic %s",
      quality_topic_.c_str());
    return false;
  }

  const double quality_age_sec = (now() - quality_wall_time).seconds();
  if (quality_age_sec > quality_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Relocalization quality topic %s is stale: age=%.3fs",
      quality_topic_.c_str(),
      quality_age_sec);
    return false;
  }

  const auto & data = quality_msg->data;
  const bool accepted = data[0] > 0.5f;
  const bool converged = data[1] > 0.5f;
  const double fitness = static_cast<double>(data[2]);
  const int consecutive_accepts = static_cast<int>(std::lround(data[9]));

  if (quality_require_accepted_ && !accepted) {
    return false;
  }
  if (quality_require_converged_ && !converged) {
    return false;
  }
  if (!std::isfinite(fitness) || fitness > quality_max_fitness_) {
    return false;
  }
  if (consecutive_accepts < quality_min_consecutive_accepts_) {
    return false;
  }

  return true;
}

bool RCSerialDriver::readPoseFromTopic(float & x, float & y, float & z, float & yaw)
{
  geometry_msgs::msg::PoseStamped::SharedPtr pose_msg;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_msg = latest_pose_;
  }

  if (!pose_msg) {
    return false;
  }

  const rclcpp::Time stamp(pose_msg->header.stamp);
  if (stamp.nanoseconds() != 0) {
    const double age_sec = (now() - stamp).seconds();
    if (age_sec > pose_topic_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Pose topic %s is stale: age=%.3fs",
        pose_topic_.c_str(),
        age_sec);
      return false;
    }
  }

  x = static_cast<float>(pose_msg->pose.position.x);
  y = static_cast<float>(pose_msg->pose.position.y);
  z = static_cast<float>(pose_msg->pose.position.z);
  yaw = yawFromQuaternion(pose_msg->pose.orientation);
  return true;
}

bool RCSerialDriver::readYawFromTopic(float & yaw)
{
  geometry_msgs::msg::PoseStamped::SharedPtr yaw_msg;
  {
    std::lock_guard<std::mutex> lock(yaw_pose_mutex_);
    yaw_msg = latest_yaw_pose_;
  }

  if (!yaw_msg) {
    return false;
  }

  const rclcpp::Time stamp(yaw_msg->header.stamp);
  if (stamp.nanoseconds() != 0) {
    const double age_sec = (now() - stamp).seconds();
    if (age_sec > yaw_pose_topic_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "CAN yaw topic %s is stale: age=%.3fs",
        yaw_pose_topic_.c_str(),
        age_sec);
      return false;
    }
  }

  yaw = yawFromQuaternion(yaw_msg->pose.orientation);
  return true;
}

bool RCSerialDriver::readPoseFromTf(float & x, float & y, float & z, float & yaw)
{
    // 低频完整 3D 坐标: robot_origin -> robot_base
    // 用它取 z
    geometry_msgs::msg::TransformStamped tf_low =
      tf_buffer_->lookupTransform("robot_origin", "robot_base", tf2::TimePointZero);

    // 高频平面坐标: robot_origin -> robot_base_highrate
    // 用它取 x, y, yaw
    geometry_msgs::msg::TransformStamped tf_high =
      tf_buffer_->lookupTransform("robot_origin", "robot_base_highrate", tf2::TimePointZero);

    // === 机器人业务 TF 坐标: x=右, y=前, z=上 ===
    x = static_cast<float>(tf_high.transform.translation.x);
    y = static_cast<float>(tf_high.transform.translation.y);

    // === z 来自低频 robot_base ===
    z = static_cast<float>(tf_low.transform.translation.z);

    yaw = yawFromQuaternion(tf_high.transform.rotation);
    return true;
}

void RCSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Reopening CAN interface...");
  if (can_socket_ >= 0) {
    close(can_socket_);
  }

  try {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      throw std::runtime_error("socket failed: " + std::string(strerror(errno)));
    }

    fcntl(can_socket_, F_SETFL, O_NONBLOCK);
    int send_buf_size = 1 << 20;
    setsockopt(can_socket_, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      close(can_socket_);
      throw std::runtime_error("ioctl failed for " + can_interface_);
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      close(can_socket_);
      throw std::runtime_error("bind failed for " + can_interface_);
    }

    applyRestartCanFilter(can_socket_, g_can_id_restart, get_logger());

    RCLCPP_INFO(get_logger(), "CAN interface reopened successfully");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to reopen CAN: %s", ex.what());
    can_socket_ = -1;
  }
}

}  // namespace rc_serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc_serial_driver::RCSerialDriver)
