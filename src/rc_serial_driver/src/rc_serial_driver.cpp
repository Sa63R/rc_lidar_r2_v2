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
#include <cstring>
#include <memory>
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
std::string g_restart_script = "/home/rc/jwt/rc/autostart/restart_lidar_stack.sh";
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
  
  g_can_id_restart = declare_parameter<int>("can_id_restart", 0x30);
  g_restart_script = declare_parameter<std::string>(
    "restart_script",
    "/home/rc/jwt/rc/autostart/restart_lidar_stack.sh");
  g_restart_enabled = declare_parameter<bool>("restart_enabled", true);

  RCLCPP_INFO(get_logger(), "Restart CAN ID: 0x%X", g_can_id_restart);
  RCLCPP_INFO(get_logger(), "Restart script: %s", g_restart_script.c_str());

  RCLCPP_INFO(get_logger(), "CAN Interface: %s", can_interface_.c_str());
  RCLCPP_INFO(get_logger(), "Expected CAN Bitrate: %d bit/s (ensure pre-configured)", can_bitrate_);
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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "CAN socket not ready, skipping send.");
    return;
  }

  try {
    // 低频完整 3D 坐标: init -> base_link
    // 用它取 z
    geometry_msgs::msg::TransformStamped tf_low =
      tf_buffer_->lookupTransform("init", "base_link", tf2::TimePointZero);

    // 高频平面坐标: init -> base_link_highrate
    // 用它取 x, y, yaw
    geometry_msgs::msg::TransformStamped tf_high =
      tf_buffer_->lookupTransform("init", "base_link_highrate", tf2::TimePointZero);

    // === x, y 来自高频 base_link_highrate ===
    float x = static_cast<float>(tf_high.transform.translation.x);
    float y = static_cast<float>(tf_high.transform.translation.y);

    // === z 来自低频 base_link ===
    float z = static_cast<float>(tf_low.transform.translation.z);

    // === yaw 来自高频 base_link_highrate ===
    tf2::Quaternion q_high(
      tf_high.transform.rotation.x,
      tf_high.transform.rotation.y,
      tf_high.transform.rotation.z,
      tf_high.transform.rotation.w);

    tf2::Matrix3x3 m_high(q_high);
    double roll_high, pitch_high, yaw_high;
    m_high.getRPY(roll_high, pitch_high, yaw_high);

    float yaw_f = static_cast<float>(yaw_high);

    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "Sending mixed pose: "
      << "x_high=" << x
      << " y_high=" << y
      << " z_low=" << z
      << " yaw_high=" << yaw_f);

    // === 帧1: x_high, y_high (ID = 0x12) ===
    struct can_frame frame1;
    frame1.can_id = can_id_frame1_;
    frame1.can_dlc = 8;
    std::memcpy(frame1.data + 0, &x, sizeof(float));
    std::memcpy(frame1.data + 4, &y, sizeof(float));

    ssize_t bytes_written = write(can_socket_, &frame1, sizeof(struct can_frame));
    if (bytes_written < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "CAN buffer full, dropping frame1");
      } else {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to send frame1: %s", strerror(errno));
        reopenPort();
        return;
      }
    }

    // === 帧2: z_low, yaw_high (ID = 0x13) ===
    struct can_frame frame2;
    frame2.can_id = can_id_frame2_;
    frame2.can_dlc = 8;
    std::memcpy(frame2.data + 0, &z, sizeof(float));
    std::memcpy(frame2.data + 4, &yaw_f, sizeof(float));

    bytes_written = write(can_socket_, &frame2, sizeof(struct can_frame));
    if (bytes_written < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
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