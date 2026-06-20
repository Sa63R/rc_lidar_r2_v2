#include <chrono>
#include <cmath>
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
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    auto_initialize_identity_ = declare_parameter<bool>("auto_initialize_identity", true);
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

    map_T_odom_.setIdentity();
    has_map_to_odom_ = auto_initialize_identity_;

    loadMap();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 20);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_topic_, 20);
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
    if (static_cast<int>(filtered_scan->size()) < min_scan_points_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "scan has too few points after filtering: %zu", filtered_scan->size());
      return;
    }

    Eigen::Matrix4f result = guess;
    double fitness = 0.0;
    bool converged = align(filtered_scan, guess, result, fitness);

    if (!converged || !std::isfinite(fitness) || fitness > fitness_threshold_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "map registration rejected: converged=%d fitness=%.3f threshold=%.3f",
          converged ? 1 : 0, fitness, fitness_threshold_);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_T_odom_ = result;
    }

    publishLocalization(stamp, odom, result, fitness);
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
      double fitness) {
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

    if (tf_broadcaster_) {
      tf_broadcaster_->sendTransform(matrixToTransform(stamp, map_frame_, odom_frame_, map_T_odom));
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "localized in map: x=%.3f y=%.3f z=%.3f fitness=%.3f",
        output.pose.pose.position.x, output.pose.pose.position.y,
        output.pose.pose.position.z, fitness);
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
  std::string map_frame_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string registration_method_;
  bool publish_tf_{true};
  bool auto_initialize_identity_{true};
  bool has_map_to_odom_{true};
  bool map_loaded_{false};
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

  CloudT::Ptr map_raw_;
  CloudT::Ptr map_filtered_;
  CloudT::Ptr latest_scan_;
  builtin_interfaces::msg::Time latest_scan_stamp_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  Eigen::Matrix4f map_T_odom_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
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
