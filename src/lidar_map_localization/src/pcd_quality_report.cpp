#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

struct Bounds {
  double min_x{std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double min_z{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};
  double max_z{-std::numeric_limits<double>::infinity()};
};

std::string key2d(const PointT& p, double leaf) {
  const auto ix = static_cast<std::int64_t>(std::floor(p.x / leaf));
  const auto iy = static_cast<std::int64_t>(std::floor(p.y / leaf));
  return std::to_string(ix) + "," + std::to_string(iy);
}

std::string key3d(const PointT& p, double leaf) {
  const auto ix = static_cast<std::int64_t>(std::floor(p.x / leaf));
  const auto iy = static_cast<std::int64_t>(std::floor(p.y / leaf));
  const auto iz = static_cast<std::int64_t>(std::floor(p.z / leaf));
  return std::to_string(ix) + "," + std::to_string(iy) + "," + std::to_string(iz);
}

double percentile(std::vector<double> values, double q) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double index = q * static_cast<double>(values.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(index));
  const auto hi = static_cast<std::size_t>(std::ceil(index));
  if (lo == hi) {
    return values[lo];
  }
  const double ratio = index - static_cast<double>(lo);
  return values[lo] * (1.0 - ratio) + values[hi] * ratio;
}

void usage() {
  std::cout << "Usage: ros2 run lidar_map_localization pcd_quality_report "
               "--map maps/mid360_map.pcd\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string map_path = "maps/mid360_map.pcd";
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if ((arg == "--map" || arg == "-m") && i + 1 < argc) {
      map_path = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      usage();
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      usage();
      return 2;
    }
  }

  CloudT cloud;
  if (pcl::io::loadPCDFile<PointT>(map_path, cloud) != 0) {
    std::cerr << "ERROR: failed to load map: " << map_path << "\n";
    return 1;
  }

  Bounds bounds;
  std::vector<double> z_values;
  z_values.reserve(cloud.size());
  std::size_t valid_points = 0;
  for (const auto& p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    ++valid_points;
    bounds.min_x = std::min(bounds.min_x, static_cast<double>(p.x));
    bounds.min_y = std::min(bounds.min_y, static_cast<double>(p.y));
    bounds.min_z = std::min(bounds.min_z, static_cast<double>(p.z));
    bounds.max_x = std::max(bounds.max_x, static_cast<double>(p.x));
    bounds.max_y = std::max(bounds.max_y, static_cast<double>(p.y));
    bounds.max_z = std::max(bounds.max_z, static_cast<double>(p.z));
    z_values.push_back(p.z);
  }

  if (valid_points == 0) {
    std::cerr << "ERROR: map has no valid finite XYZ points: " << map_path << "\n";
    return 1;
  }

  std::cout << "PCD quality report\n";
  std::cout << "map: " << map_path << "\n";
  std::cout << "raw_points: " << cloud.size() << "\n";
  std::cout << "valid_points: " << valid_points << "\n";
  std::cout << "range_x_m: [" << bounds.min_x << ", " << bounds.max_x << "] span="
            << bounds.max_x - bounds.min_x << "\n";
  std::cout << "range_y_m: [" << bounds.min_y << ", " << bounds.max_y << "] span="
            << bounds.max_y - bounds.min_y << "\n";
  std::cout << "range_z_m: [" << bounds.min_z << ", " << bounds.max_z << "] span="
            << bounds.max_z - bounds.min_z << "\n";
  std::cout << "z_percentiles_m: p01=" << percentile(z_values, 0.01)
            << " p50=" << percentile(z_values, 0.50)
            << " p99=" << percentile(z_values, 0.99) << "\n";

  for (double leaf : {0.05, 0.10, 0.20}) {
    std::unordered_set<std::string> voxels;
    voxels.reserve(valid_points);
    for (const auto& p : cloud) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        voxels.insert(key3d(p, leaf));
      }
    }
    const double points_per_voxel =
        voxels.empty() ? 0.0 : static_cast<double>(valid_points) / voxels.size();
    std::cout << "voxel_" << leaf << "m: occupied=" << voxels.size()
              << " points_per_voxel=" << points_per_voxel << "\n";
  }

  constexpr double xy_leaf = 0.10;
  std::unordered_set<std::string> xy_cells;
  xy_cells.reserve(valid_points);
  for (const auto& p : cloud) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      xy_cells.insert(key2d(p, xy_leaf));
    }
  }
  const double span_x = std::max(bounds.max_x - bounds.min_x, xy_leaf);
  const double span_y = std::max(bounds.max_y - bounds.min_y, xy_leaf);
  const auto bbox_cells =
      static_cast<std::size_t>(std::ceil(span_x / xy_leaf) * std::ceil(span_y / xy_leaf));
  const double xy_fill_ratio =
      bbox_cells == 0 ? 0.0 : static_cast<double>(xy_cells.size()) / bbox_cells;
  const double xy_points_per_cell =
      xy_cells.empty() ? 0.0 : static_cast<double>(valid_points) / xy_cells.size();
  std::cout << "xy_grid_0.10m: occupied=" << xy_cells.size()
            << " bbox_cells=" << bbox_cells
            << " fill_ratio=" << xy_fill_ratio
            << " points_per_cell=" << xy_points_per_cell << "\n";

  std::vector<std::string> warnings;
  if (valid_points < 100000) {
    warnings.push_back("point count is low for high-precision relocalization");
  }
  if (xy_points_per_cell < 5.0) {
    warnings.push_back("10cm XY cells are sparse; map may be noisy or under-scanned");
  }
  if ((percentile(z_values, 0.99) - percentile(z_values, 0.01)) < 1.0) {
    warnings.push_back("vertical structure is weak; pure flat maps localize less robustly");
  }
  if (xy_fill_ratio < 0.01) {
    warnings.push_back("large bounding box with sparse occupied cells; check for long drifting tails");
  }

  if (warnings.empty()) {
    std::cout << "verdict: OK - map density looks usable. Compare fitness/jitter live for final judgment.\n";
  } else {
    std::cout << "verdict: CHECK\n";
    for (const auto& warning : warnings) {
      std::cout << "- " << warning << "\n";
    }
  }

  return 0;
}
