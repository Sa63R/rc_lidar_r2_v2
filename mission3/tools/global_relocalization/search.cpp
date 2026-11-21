// Automatic coarse alignment of a partial live cloud to a saved map.
// Uses PCL's public FPFH + SampleConsensusPrerejective implementation, followed
// by ICP. No ROS, no manually supplied pose, no modification of either input.
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <nlohmann/json.hpp>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using P = pcl::PointXYZ;
using Cloud = pcl::PointCloud<P>;
using Features = pcl::PointCloud<pcl::FPFHSignature33>;
using json = nlohmann::json;

Cloud::Ptr downsample(const Cloud::Ptr& raw, float leaf) {
  pcl::VoxelGrid<P> filter;
  filter.setLeafSize(leaf, leaf, leaf);
  filter.setInputCloud(raw);
  auto out = Cloud::Ptr(new Cloud);
  filter.filter(*out);
  return out;
}

struct Prepared {
  Cloud::Ptr points;
  Features::Ptr features;
};

Prepared describe(const Cloud::Ptr& raw, float leaf) {
  auto points = downsample(raw, leaf);
  auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<P, pcl::Normal> normal(2);
  normal.setInputCloud(points);
  normal.setRadiusSearch(leaf * 3.f);
  normal.compute(*normals);
  auto features = Features::Ptr(new Features);
  pcl::FPFHEstimationOMP<P, pcl::Normal, pcl::FPFHSignature33> fpfh(2);
  fpfh.setInputCloud(points);
  fpfh.setInputNormals(normals);
  fpfh.setRadiusSearch(leaf * 5.f);
  fpfh.compute(*features);
  Prepared result{Cloud::Ptr(new Cloud), Features::Ptr(new Features)};
  for (size_t i = 0; i < points->size(); ++i) {
    bool finite = true;
    for (float v : (*features)[i].histogram) finite = finite && std::isfinite(v);
    if (finite && std::isfinite((*normals)[i].normal_x)) {
      result.points->push_back((*points)[i]);
      result.features->push_back((*features)[i]);
    }
  }
  return result;
}

Cloud::Ptr load(const std::string& path) {
  auto raw = Cloud::Ptr(new Cloud);
  if (pcl::io::loadPCDFile(path, *raw) != 0) throw std::runtime_error("cannot read " + path);
  auto finite = Cloud::Ptr(new Cloud);
  for (const auto& p : *raw) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) finite->push_back(p);
  }
  if (finite->size() < 200) throw std::runtime_error("too few finite points in " + path);
  return finite;
}

json assess(const Cloud::Ptr& validation, const Cloud::Ptr& map,
            const Eigen::Matrix4f& T, float radius) {
  pcl::KdTreeFLANN<P> tree;
  tree.setInputCloud(map);
  Cloud transformed;
  pcl::transformPointCloud(*validation, transformed, T);
  std::vector<float> distances;
  std::vector<int> index(1);
  std::vector<float> squared(1);
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d outer = Eigen::Matrix3d::Zero();
  for (const auto& p : transformed) {
    if (tree.nearestKSearch(p, 1, index, squared) > 0 && squared[0] <= radius * radius) {
      distances.push_back(std::sqrt(squared[0]));
      Eigen::Vector3d x(p.x, p.y, p.z);
      sum += x;
      outer += x * x.transpose();
    }
  }
  double rms = 0.;
  for (auto d : distances) rms += d * d;
  json result{{"validation_points", transformed.size()}, {"inliers", distances.size()},
              {"overlap", double(distances.size()) / std::max<size_t>(1, transformed.size())}};
  if (distances.empty()) {
    result["inlier_rms_m"] = nullptr;
    result["support_eigenvalues"] = {0., 0., 0.};
  } else {
    result["inlier_rms_m"] = std::sqrt(rms / distances.size());
    auto mean = sum / distances.size();
    Eigen::Matrix3d covariance = outer / distances.size() - mean * mean.transpose();
    auto e = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(covariance).eigenvalues();
    result["support_eigenvalues"] = {e[0], e[1], e[2]};
  }
  return result;
}

int main(int argc, char** argv) {
  try {
    if (argc != 6) throw std::runtime_error("usage: search map.pcd source.pcd heldout.pcd out.json iterations");
    auto start = std::chrono::steady_clock::now();
    const int iterations = std::stoi(argv[5]);
    if (iterations < 1000 || iterations > 500000) throw std::runtime_error("iterations out of range");
    auto map = load(argv[1]), source = load(argv[2]), heldout = load(argv[3]);
    const float leaf = .25f;
    auto target = describe(map, leaf), moving = describe(source, leaf);
    auto fine_map = downsample(map, .12f), fine_source = downsample(source, .12f);
    auto validation = downsample(heldout, .15f);
    if (moving.points->size() < 100 || target.points->size() < 100)
      throw std::runtime_error("insufficient valid geometric features");
    json output{{"method", "PCL FPFH/prerejective RANSAC + ICP"},
                {"source_features", moving.points->size()}, {"map_features", target.points->size()},
                {"candidates", json::array()}, {"global_uniqueness_proven", false}};
    // Several independent searches, then a search excluding the best matched
    // surface patch to look for a geographically different competing location.
    Eigen::Matrix4f best = Eigen::Matrix4f::Identity();
    double best_overlap = -1.;
    for (int trial = 0; trial < 4; ++trial) {
      Prepared search_target = target;
      if (trial == 3 && best_overlap > 0.) {
        Cloud aligned;
        pcl::transformPointCloud(*moving.points, aligned, best);
        auto aligned_ptr = aligned.makeShared();
        pcl::KdTreeFLANN<P> matched_tree;
        matched_tree.setInputCloud(aligned_ptr);
        search_target = {Cloud::Ptr(new Cloud), Features::Ptr(new Features)};
        std::vector<int> ids(1); std::vector<float> d2(1);
        for (size_t i = 0; i < target.points->size(); ++i) {
          if (matched_tree.nearestKSearch((*target.points)[i], 1, ids, d2) && d2[0] > 1.f) {
            search_target.points->push_back((*target.points)[i]);
            search_target.features->push_back((*target.features)[i]);
          }
        }
        if (search_target.points->size() < 100) continue;
      }
      std::srand(1234 + trial * 177);
      pcl::SampleConsensusPrerejective<P, P, pcl::FPFHSignature33> align;
      align.setInputSource(moving.points); align.setSourceFeatures(moving.features);
      align.setInputTarget(search_target.points); align.setTargetFeatures(search_target.features);
      align.setMaximumIterations(iterations); align.setNumberOfSamples(3);
      align.setCorrespondenceRandomness(10); align.setSimilarityThreshold(.90f);
      align.setMaxCorrespondenceDistance(.50f); align.setInlierFraction(.35f);
      Cloud aligned;
      align.align(aligned);
      if (!align.hasConverged()) continue;
      auto coarse = align.getFinalTransformation();
      // Both LIO worlds should be approximately gravity aligned. Reject upside
      // down floor/wall hypotheses before any local refinement.
      if (coarse(2, 2) < std::cos(15.f * float(M_PI) / 180.f)) continue;
      pcl::IterativeClosestPoint<P, P> icp;
      icp.setInputSource(fine_source); icp.setInputTarget(fine_map);
      icp.setMaxCorrespondenceDistance(.45); icp.setMaximumIterations(60);
      icp.setTransformationEpsilon(1.e-7); icp.align(aligned, coarse);
      if (!icp.hasConverged()) continue;
      auto T = icp.getFinalTransformation();
      if (!T.allFinite() || T(2, 2) < std::cos(15.f * float(M_PI) / 180.f)) continue;
      json candidate = assess(validation, fine_map, T, .25f);
      candidate["trial"] = trial;
      candidate["map_T_camera_init"] = json::array();
      for (int i = 0; i < 4; ++i)
        candidate["map_T_camera_init"].push_back({T(i,0), T(i,1), T(i,2), T(i,3)});
      candidate["icp_fitness"] = icp.getFitnessScore();
      output["candidates"].push_back(candidate);
      if (candidate["overlap"].get<double>() > best_overlap) {
        best_overlap = candidate["overlap"].get<double>(); best = T;
      }
    }
    output["elapsed_sec"] = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    output["physical_accuracy_verified"] = false;
    std::ofstream file(argv[4]);
    file << output.dump(2) << '\n';
    std::cout << output.dump(2) << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 2;
  }
}
