#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

namespace lidar_map_localization {

class MapLocalizer : public rclcpp::Node {
 public:
  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;

  MapLocalizer() : Node("map_localizer") {
    map_path_ = declare_parameter<std::string>("map_path", "/home/orangepi/RC_LIDAR_NOW/maps/mid360_map.pcd");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/cloud_registered");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", "/initialpose");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/map_lidar/odom");
    output_pose_topic_ = declare_parameter<std::string>("output_pose_topic", "/map_lidar/pose");
    output_map_topic_ = declare_parameter<std::string>("output_map_topic", "/localization/map");
    output_quality_topic_ = declare_parameter<std::string>("output_quality_topic", "/map_lidar/quality");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    auto_initialize_identity_ = declare_parameter<bool>("auto_initialize_identity", true);
    initial_guess_enabled_ = declare_parameter<bool>("initial_guess_enabled", false);
    initial_guess_x_ = declare_parameter<double>("initial_guess_x", 0.0);
    initial_guess_y_ = declare_parameter<double>("initial_guess_y", 0.0);
    initial_guess_z_ = declare_parameter<double>("initial_guess_z", 0.0);
    initial_guess_roll_ = declare_parameter<double>("initial_guess_roll", 0.0);
    initial_guess_pitch_ = declare_parameter<double>("initial_guess_pitch", 0.0);
    initial_guess_yaw_ = declare_parameter<double>("initial_guess_yaw", 0.0);
    registration_method_ = declare_parameter<std::string>("registration_method", "ndt");
    localize_period_sec_ = declare_parameter<double>("localize_period_sec", 0.5);
    map_leaf_size_ = declare_parameter<double>("map_leaf_size", 0.25);
    scan_leaf_size_ = declare_parameter<double>("scan_leaf_size", 0.25);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 80);
    fitness_threshold_ = declare_parameter<double>("fitness_threshold", 3.0);
    ndt_resolution_ = declare_parameter<double>("ndt_resolution", 1.0);
    ndt_step_size_ = declare_parameter<double>("ndt_step_size", 0.1);
    ndt_transformation_epsilon_ = declare_parameter<double>("ndt_transformation_epsilon", 0.01);
    ndt_max_iterations_ = declare_parameter<int>("ndt_max_iterations", 35);
    icp_max_correspondence_distance_ = declare_parameter<double>("icp_max_correspondence_distance", 1.5);
    icp_transformation_epsilon_ = declare_parameter<double>("icp_transformation_epsilon", 0.001);
    icp_euclidean_fitness_epsilon_ = declare_parameter<double>("icp_euclidean_fitness_epsilon", 0.01);
    icp_max_iterations_ = declare_parameter<int>("icp_max_iterations", 40);
    quality_overlap_enabled_ = declare_parameter<bool>("quality_overlap_enabled", false);
    quality_overlap_distance_ = declare_parameter<double>("quality_overlap_distance", 0.20);
    quality_overlap_period_sec_ = declare_parameter<double>("quality_overlap_period_sec", 1.0);
    quality_overlap_max_points_ = declare_parameter<int>("quality_overlap_max_points", 1000);

    map_T_odom_.setIdentity();
    if (initial_guess_enabled_) {
      map_T_odom_ = xyzRpyToMatrix(
          initial_guess_x_, initial_guess_y_, initial_guess_z_,
          initial_guess_roll_, initial_guess_pitch_, initial_guess_yaw_);
      has_map_to_odom_ = true;
      RCLCPP_INFO(
          get_logger(),
          "using initial map->%s guess: xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
          odom_frame_.c_str(), initial_guess_x_, initial_guess_y_, initial_guess_z_,
          initial_guess_roll_, initial_guess_pitch_, initial_guess_yaw_);
    } else {
      has_map_to_odom_ = auto_initialize_identity_;
      if (auto_initialize_identity_) {
        RCLCPP_WARN(
            get_logger(),
            "auto_initialize_identity is true; assuming current FAST-LIO origin is near the saved map origin");
      } else {
        RCLCPP_WARN(
            get_logger(),
            "no initial map->%s guess; waiting for /initialpose or RC_RELOCALIZATION_INITIAL_GUESS=1",
            odom_frame_.c_str());
      }
    }

    loadMap();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 20);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_topic_, 20);
    quality_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_quality_topic_, 20);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_map_topic_, rclcpp::QoS(1).transient_local().reliable());

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&MapLocalizer::scanCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 20,
        std::bind(&MapLocalizer::odomCallback, this, std::placeholders::_1));
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initialpose_topic_, 10,
        std::bind(&MapLocalizer::initialPoseCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(localize_period_sec_)),
        std::bind(&MapLocalizer::localizeTimerCallback, this));

    publishMap();
    RCLCPP_INFO(
        get_logger(),
        "map localizer ready: map=%s scan=%s odom=%s output=%s",
        map_path_.c_str(), scan_topic_.c_str(), odom_topic_.c_str(),
        output_odom_topic_.c_str());
  }

 private:
  enum class QualityReason {
    kAccepted = 0,
    kTooFewPoints = 1,
    kNotConverged = 2,
    kBadFitness = 3,
  };

  void loadMap() {
    map_raw_.reset(new CloudT);
    map_filtered_.reset(new CloudT);

    if (pcl::io::loadPCDFile<PointT>(map_path_, *map_raw_) != 0) {
      RCLCPP_ERROR(get_logger(), "failed to load map: %s", map_path_.c_str());
      map_loaded_ = false;
      return;
    }

    removeInvalidPoints(map_raw_);
    map_filtered_ = downsample(map_raw_, map_leaf_size_);
    map_loaded_ = !map_filtered_->empty();

    if (map_loaded_) {
      map_kdtree_.reset(new pcl::KdTreeFLANN<PointT>);
      map_kdtree_->setInputCloud(map_filtered_);
      RCLCPP_INFO(
          get_logger(), "loaded map: raw=%zu filtered=%zu path=%s",
          map_raw_->size(), map_filtered_->size(), map_path_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "map is empty after filtering: %s", map_path_.c_str());
    }
  }

  static void removeInvalidPoints(CloudT::Ptr cloud) {
    CloudT valid;
    valid.reserve(cloud->size());
    for (const auto& point : *cloud) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        valid.push_back(point);
      }
    }
    cloud->swap(valid);
  }

  CloudT::Ptr downsample(const CloudT::Ptr& input, double leaf_size) const {
    if (leaf_size <= 0.0) {
      return input;
    }

    CloudT::Ptr output(new CloudT);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(
        static_cast<float>(leaf_size),
        static_cast<float>(leaf_size),
        static_cast<float>(leaf_size));
    voxel.filter(*output);
    return output;
  }

  void publishMap() {
    if (!map_loaded_) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_filtered_, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    map_pub_->publish(msg);
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    removeInvalidPoints(cloud);

    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = cloud;
    latest_scan_stamp_ = msg->header.stamp;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = msg;
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::Matrix4f map_T_body = poseToMatrix(msg->pose.pose);
    Eigen::Matrix4f odom_T_body = Eigen::Matrix4f::Identity();
    if (latest_odom_) {
      odom_T_body = poseToMatrix(latest_odom_->pose.pose);
    }
    map_T_odom_ = map_T_body * odom_T_body.inverse();
    has_map_to_odom_ = true;
    RCLCPP_INFO(get_logger(), "initial pose received; map->%s guess updated", odom_frame_.c_str());
  }

  void localizeTimerCallback() {
    CloudT::Ptr scan;
    nav_msgs::msg::Odometry::SharedPtr odom;
    rclcpp::Time stamp;
    Eigen::Matrix4f guess;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_loaded_ || !latest_scan_ || !latest_odom_) {
        return;
      }
      if (!has_map_to_odom_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "waiting for an initial pose because auto_initialize_identity is false");
        return;
      }
      scan = latest_scan_;
      odom = latest_odom_;
      stamp = rclcpp::Time(latest_scan_stamp_);
      guess = map_T_odom_;
    }

    CloudT::Ptr filtered_scan = downsample(scan, scan_leaf_size_);
    const double scan_age_sec = ageSec(now(), stamp);
    const double odom_age_sec = ageSec(now(), rclcpp::Time(odom->header.stamp));
    const std::size_t raw_scan_points = scan->size();
    if (static_cast<int>(filtered_scan->size()) < min_scan_points_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "scan has too few points after filtering: %zu", filtered_scan->size());
      ++consecutive_rejects_;
      consecutive_accepts_ = 0;
      publishQuality(
          stamp, false, false, -1.0, filtered_scan->size(), 0.0,
          QualityReason::kTooFewPoints, 0.0, raw_scan_points,
          scan_age_sec, odom_age_sec, 0.0, 0.0, 0.0, -1.0);
      return;
    }

    Eigen::Matrix4f result = guess;
    double fitness = 0.0;
    const auto align_start = std::chrono::steady_clock::now();
    bool converged = align(filtered_scan, guess, result, fitness);
    const auto align_end = std::chrono::steady_clock::now();
    const double align_ms = std::chrono::duration<double, std::milli>(
        align_end - align_start).count();
    const double map_to_odom_delta_xy = std::hypot(
        static_cast<double>(result(0, 3) - guess(0, 3)),
        static_cast<double>(result(1, 3) - guess(1, 3)));
    const double map_to_odom_delta_yaw_deg = std::abs(
        radiansToDegrees(shortestAngle(yawFromMatrix(result) - yawFromMatrix(guess))));
    const double overlap_ratio = computeOverlapRatio(filtered_scan, result);

    if (!converged || !std::isfinite(fitness) || fitness > fitness_threshold_) {
      ++consecutive_rejects_;
      consecutive_accepts_ = 0;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "map registration rejected: converged=%d fitness=%.3f threshold=%.3f",
          converged ? 1 : 0, fitness, fitness_threshold_);
      publishQuality(
          stamp, converged, false, fitness, filtered_scan->size(), 0.0,
          converged ? QualityReason::kBadFitness : QualityReason::kNotConverged,
          align_ms, raw_scan_points, scan_age_sec, odom_age_sec, 0.0,
          map_to_odom_delta_xy, map_to_odom_delta_yaw_deg, overlap_ratio);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_T_odom_ = result;
    }

    publishLocalization(
        stamp, odom, result, fitness, filtered_scan->size(), align_ms,
        raw_scan_points, scan_age_sec, odom_age_sec, map_to_odom_delta_xy,
        map_to_odom_delta_yaw_deg, overlap_ratio);
  }

  bool align(
      const CloudT::Ptr& scan,
      const Eigen::Matrix4f& guess,
      Eigen::Matrix4f& result,
      double& fitness) {
    CloudT aligned;
    if (registration_method_ == "icp") {
      pcl::IterativeClosestPoint<PointT, PointT> icp;
      icp.setInputTarget(map_filtered_);
      icp.setInputSource(scan);
      icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
      icp.setTransformationEpsilon(icp_transformation_epsilon_);
      icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);
      icp.setMaximumIterations(icp_max_iterations_);
      icp.align(aligned, guess);
      result = icp.getFinalTransformation();
      fitness = icp.getFitnessScore();
      return icp.hasConverged();
    }

    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setInputTarget(map_filtered_);
    ndt.setInputSource(scan);
    ndt.setResolution(ndt_resolution_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt.setMaximumIterations(ndt_max_iterations_);
    ndt.align(aligned, guess);
    result = ndt.getFinalTransformation();
    fitness = ndt.getFitnessScore();
    return ndt.hasConverged();
  }

  void publishLocalization(
      const rclcpp::Time& stamp,
      const nav_msgs::msg::Odometry::SharedPtr& odom,
      const Eigen::Matrix4f& map_T_odom,
      double fitness,
      std::size_t scan_points,
      double align_ms,
      std::size_t raw_scan_points,
      double scan_age_sec,
      double odom_age_sec,
      double map_to_odom_delta_xy,
      double map_to_odom_delta_yaw_deg,
      double overlap_ratio) {
    Eigen::Matrix4f odom_T_body = poseToMatrix(odom->pose.pose);
    Eigen::Matrix4f map_T_body = map_T_odom * odom_T_body;

    nav_msgs::msg::Odometry output;
    output.header.stamp = stamp;
    output.header.frame_id = map_frame_;
    output.child_frame_id = body_frame_;
    output.pose.pose = matrixToPose(map_T_body);
    output.pose.covariance = odom->pose.covariance;
    output.twist = odom->twist;
    odom_pub_->publish(output);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = output.header;
    pose.pose = output.pose.pose;
    pose_pub_->publish(pose);

    const double x = output.pose.pose.position.x;
    const double y = output.pose.pose.position.y;
    const double yaw = yawFromMatrix(map_T_body);
    double jump_xy = 0.0;
    double jump_yaw_deg = 0.0;
    if (last_output_pose_available_) {
      jump_xy = std::hypot(x - last_output_x_, y - last_output_y_);
      jump_yaw_deg = std::abs(
          radiansToDegrees(shortestAngle(yaw - last_output_yaw_)));
    }
    last_output_x_ = x;
    last_output_y_ = y;
    last_output_yaw_ = yaw;
    last_output_pose_available_ = true;
    consecutive_rejects_ = 0;
    ++consecutive_accepts_;
    publishQuality(
        stamp, true, true, fitness, scan_points, jump_xy,
        QualityReason::kAccepted, align_ms, raw_scan_points,
        scan_age_sec, odom_age_sec, jump_yaw_deg, map_to_odom_delta_xy,
        map_to_odom_delta_yaw_deg, overlap_ratio);

    if (tf_broadcaster_) {
      tf_broadcaster_->sendTransform(matrixToTransform(stamp, map_frame_, odom_frame_, map_T_odom));
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "localized in map: x=%.3f y=%.3f z=%.3f fitness=%.3f",
        output.pose.pose.position.x, output.pose.pose.position.y,
        output.pose.pose.position.z, fitness);
  }

  double computeOverlapRatio(const CloudT::Ptr& scan, const Eigen::Matrix4f& transform) {
    if (!quality_overlap_enabled_ || !map_kdtree_ || !map_filtered_ || map_filtered_->empty()) {
      return -1.0;
    }
    const double current_sec = now().seconds();
    if (
        last_overlap_time_sec_ >= 0.0
        && current_sec - last_overlap_time_sec_ < quality_overlap_period_sec_) {
      return last_overlap_ratio_;
    }
    last_overlap_time_sec_ = current_sec;

    CloudT aligned;
    pcl::transformPointCloud(*scan, aligned, transform);
    if (aligned.empty()) {
      last_overlap_ratio_ = 0.0;
      return last_overlap_ratio_;
    }

    const int max_points = std::max(1, quality_overlap_max_points_);
    const std::size_t step = std::max<std::size_t>(1, aligned.size() / max_points);
    const float max_distance_sq = static_cast<float>(
        quality_overlap_distance_ * quality_overlap_distance_);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    std::size_t checked = 0;
    std::size_t matched = 0;
    for (std::size_t i = 0; i < aligned.size(); i += step) {
      const auto& point = aligned[i];
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (map_kdtree_->nearestKSearch(point, 1, indices, distances) > 0) {
        if (!distances.empty() && distances[0] <= max_distance_sq) {
          ++matched;
        }
        ++checked;
      }
    }
    last_overlap_ratio_ = checked > 0
        ? static_cast<double>(matched) / static_cast<double>(checked)
        : 0.0;
    return last_overlap_ratio_;
  }

  void publishQuality(
      const rclcpp::Time& stamp,
      bool converged,
      bool accepted,
      double fitness,
      std::size_t scan_points,
      double jump_xy,
      QualityReason reason,
      double align_ms,
      std::size_t raw_scan_points,
      double scan_age_sec,
      double odom_age_sec,
      double output_jump_yaw_deg,
      double map_to_odom_delta_xy,
      double map_to_odom_delta_yaw_deg,
      double overlap_ratio) {
    (void)stamp;
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {
        accepted ? 1.0f : 0.0f,
        converged ? 1.0f : 0.0f,
        static_cast<float>(fitness),
        static_cast<float>(fitness_threshold_),
        static_cast<float>(consecutive_rejects_),
        static_cast<float>(scan_points),
        static_cast<float>(map_filtered_ ? map_filtered_->size() : 0),
        static_cast<float>(jump_xy),
        has_map_to_odom_ ? 1.0f : 0.0f,
        static_cast<float>(consecutive_accepts_),
        static_cast<float>(static_cast<int>(reason)),
        static_cast<float>(align_ms),
        static_cast<float>(raw_scan_points),
        static_cast<float>(scan_age_sec),
        static_cast<float>(odom_age_sec),
        static_cast<float>(output_jump_yaw_deg),
        static_cast<float>(map_to_odom_delta_xy),
        static_cast<float>(map_to_odom_delta_yaw_deg),
        static_cast<float>(overlap_ratio),
    };
    quality_pub_->publish(msg);
  }

  static double ageSec(const rclcpp::Time& current, const rclcpp::Time& stamp) {
    if (stamp.nanoseconds() <= 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return (current - stamp).seconds();
  }

  static double yawFromMatrix(const Eigen::Matrix4f& matrix) {
    return std::atan2(static_cast<double>(matrix(1, 0)), static_cast<double>(matrix(0, 0)));
  }

  static double shortestAngle(double value) {
    constexpr double pi = 3.14159265358979323846;
    while (value > pi) {
      value -= 2.0 * pi;
    }
    while (value < -pi) {
      value += 2.0 * pi;
    }
    return value;
  }

  static double radiansToDegrees(double value) {
    constexpr double pi = 3.14159265358979323846;
    return value * 180.0 / pi;
  }

  static Eigen::Matrix4f poseToMatrix(const geometry_msgs::msg::Pose& pose) {
    Eigen::Quaternionf q(
        static_cast<float>(pose.orientation.w),
        static_cast<float>(pose.orientation.x),
        static_cast<float>(pose.orientation.y),
        static_cast<float>(pose.orientation.z));
    if (q.norm() == 0.0f) {
      q = Eigen::Quaternionf::Identity();
    } else {
      q.normalize();
    }

    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
    matrix(0, 3) = static_cast<float>(pose.position.x);
    matrix(1, 3) = static_cast<float>(pose.position.y);
    matrix(2, 3) = static_cast<float>(pose.position.z);
    return matrix;
  }

  static Eigen::Matrix4f xyzRpyToMatrix(
      double x, double y, double z, double roll, double pitch, double yaw) {
    Eigen::AngleAxisf roll_angle(static_cast<float>(roll), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_angle(static_cast<float>(pitch), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_angle(static_cast<float>(yaw), Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q = yaw_angle * pitch_angle * roll_angle;
    q.normalize();

    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
    matrix(0, 3) = static_cast<float>(x);
    matrix(1, 3) = static_cast<float>(y);
    matrix(2, 3) = static_cast<float>(z);
    return matrix;
  }

  static geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4f& matrix) {
    geometry_msgs::msg::Pose pose;
    Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotation);
    q.normalize();
    pose.position.x = matrix(0, 3);
    pose.position.y = matrix(1, 3);
    pose.position.z = matrix(2, 3);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  static geometry_msgs::msg::TransformStamped matrixToTransform(
      const rclcpp::Time& stamp,
      const std::string& parent_frame,
      const std::string& child_frame,
      const Eigen::Matrix4f& matrix) {
    geometry_msgs::msg::TransformStamped transform;
    geometry_msgs::msg::Pose pose = matrixToPose(matrix);
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation = pose.orientation;
    return transform;
  }

  std::mutex mutex_;
  std::string map_path_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string initialpose_topic_;
  std::string output_odom_topic_;
  std::string output_pose_topic_;
  std::string output_map_topic_;
  std::string output_quality_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string registration_method_;
  bool publish_tf_{true};
  bool auto_initialize_identity_{true};
  bool initial_guess_enabled_{false};
  bool has_map_to_odom_{true};
  bool map_loaded_{false};
  double initial_guess_x_{0.0};
  double initial_guess_y_{0.0};
  double initial_guess_z_{0.0};
  double initial_guess_roll_{0.0};
  double initial_guess_pitch_{0.0};
  double initial_guess_yaw_{0.0};
  double localize_period_sec_{0.5};
  double map_leaf_size_{0.25};
  double scan_leaf_size_{0.25};
  int min_scan_points_{80};
  double fitness_threshold_{3.0};
  double ndt_resolution_{1.0};
  double ndt_step_size_{0.1};
  double ndt_transformation_epsilon_{0.01};
  int ndt_max_iterations_{35};
  double icp_max_correspondence_distance_{1.5};
  double icp_transformation_epsilon_{0.001};
  double icp_euclidean_fitness_epsilon_{0.01};
  int icp_max_iterations_{40};
  bool quality_overlap_enabled_{false};
  double quality_overlap_distance_{0.20};
  double quality_overlap_period_sec_{1.0};
  int quality_overlap_max_points_{1000};
  double last_overlap_time_sec_{-1.0};
  double last_overlap_ratio_{-1.0};
  int consecutive_rejects_{0};
  int consecutive_accepts_{0};
  bool last_output_pose_available_{false};
  double last_output_x_{0.0};
  double last_output_y_{0.0};
  double last_output_yaw_{0.0};

  CloudT::Ptr map_raw_;
  CloudT::Ptr map_filtered_;
  pcl::KdTreeFLANN<PointT>::Ptr map_kdtree_;
  CloudT::Ptr latest_scan_;
  builtin_interfaces::msg::Time latest_scan_stamp_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  Eigen::Matrix4f map_T_odom_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr quality_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lidar_map_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_map_localization::MapLocalizer>());
  rclcpp::shutdown();
  return 0;
}
