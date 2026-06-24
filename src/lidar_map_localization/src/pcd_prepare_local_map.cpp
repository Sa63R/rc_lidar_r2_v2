// Prepare a smaller local competition map from a large PCD map.

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace
{
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

struct Options
{
  std::string input;
  std::string output;
  double leaf = 0.10;
  double margin = 0.0;
  bool force = false;
  std::optional<double> min_x;
  std::optional<double> max_x;
  std::optional<double> min_y;
  std::optional<double> max_y;
  std::optional<double> min_z;
  std::optional<double> max_z;
  std::optional<double> center_x;
  std::optional<double> center_y;
  std::optional<double> radius;
};

struct Bounds
{
  double min_x = -std::numeric_limits<double>::infinity();
  double max_x = std::numeric_limits<double>::infinity();
  double min_y = -std::numeric_limits<double>::infinity();
  double max_y = std::numeric_limits<double>::infinity();
  double min_z = -std::numeric_limits<double>::infinity();
  double max_z = std::numeric_limits<double>::infinity();
  std::string mode = "none";
};

bool fileExists(const std::string & path)
{
  struct stat buffer {};
  return stat(path.c_str(), &buffer) == 0;
}

std::string timestampString()
{
  const std::time_t now = std::time(nullptr);
  std::tm tm {};
  localtime_r(&now, &tm);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &tm);
  return std::string(buffer);
}

std::string addSuffixBeforeExtension(const std::string & path, const std::string & suffix)
{
  const std::size_t slash = path.find_last_of('/');
  const std::size_t dot = path.find_last_of('.');
  if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) {
    return path + suffix;
  }
  return path.substr(0, dot) + suffix + path.substr(dot);
}

std::string chooseOutputPath(const std::string & requested, bool force)
{
  if (force || !fileExists(requested)) {
    return requested;
  }

  const std::string stamp = timestampString();
  for (int i = 0; i < 1000; ++i) {
    std::ostringstream suffix;
    suffix << "_" << stamp;
    if (i > 0) {
      suffix << "_" << i;
    }
    const std::string candidate = addSuffixBeforeExtension(requested, suffix.str());
    if (!fileExists(candidate)) {
      return candidate;
    }
  }

  throw std::runtime_error("Unable to find a non-existing output path for " + requested);
}

double parseDouble(const std::string & value, const std::string & name)
{
  try {
    std::size_t consumed = 0;
    const double result = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(result)) {
      throw std::invalid_argument("not finite");
    }
    return result;
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
}

void printHelp()
{
  std::cout
    << "Usage:\n"
    << "  pcd_prepare_local_map --input full.pcd --output local.pcd [options]\n\n"
    << "Options:\n"
    << "  --input PATH       Input PCD map.\n"
    << "  --output PATH      Output local PCD map. Existing paths auto-rename unless --force is set.\n"
    << "  --leaf M           Voxel leaf size in meters. Default: 0.10.\n"
    << "  --force            Overwrite output path.\n"
    << "  --min-x V --max-x V --min-y V --max-y V  Crop rectangle in map frame.\n"
    << "  --min-z V --max-z V                       Optional height crop.\n"
    << "  --center-x V --center-y V --radius V      Circular XY crop in map frame.\n"
    << "  --margin M         Expand XY rectangle or radius by M meters.\n";
}

Options parseArgs(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto requireValue = [&](const std::string & name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + name);
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      printHelp();
      std::exit(0);
    } else if (arg == "--input") {
      options.input = requireValue(arg);
    } else if (arg == "--output") {
      options.output = requireValue(arg);
    } else if (arg == "--leaf") {
      options.leaf = parseDouble(requireValue(arg), arg);
    } else if (arg == "--margin") {
      options.margin = parseDouble(requireValue(arg), arg);
    } else if (arg == "--force") {
      options.force = true;
    } else if (arg == "--min-x") {
      options.min_x = parseDouble(requireValue(arg), arg);
    } else if (arg == "--max-x") {
      options.max_x = parseDouble(requireValue(arg), arg);
    } else if (arg == "--min-y") {
      options.min_y = parseDouble(requireValue(arg), arg);
    } else if (arg == "--max-y") {
      options.max_y = parseDouble(requireValue(arg), arg);
    } else if (arg == "--min-z") {
      options.min_z = parseDouble(requireValue(arg), arg);
    } else if (arg == "--max-z") {
      options.max_z = parseDouble(requireValue(arg), arg);
    } else if (arg == "--center-x") {
      options.center_x = parseDouble(requireValue(arg), arg);
    } else if (arg == "--center-y") {
      options.center_y = parseDouble(requireValue(arg), arg);
    } else if (arg == "--radius") {
      options.radius = parseDouble(requireValue(arg), arg);
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (options.input.empty() || options.output.empty()) {
    throw std::runtime_error("--input and --output are required");
  }
  if (options.leaf < 0.0) {
    throw std::runtime_error("--leaf must be >= 0");
  }
  if (options.margin < 0.0) {
    throw std::runtime_error("--margin must be >= 0");
  }

  const bool has_radius = options.center_x || options.center_y || options.radius;
  if (has_radius && !(options.center_x && options.center_y && options.radius)) {
    throw std::runtime_error("--center-x, --center-y and --radius must be provided together");
  }
  if (options.radius && *options.radius <= 0.0) {
    throw std::runtime_error("--radius must be > 0");
  }
  const bool has_rect = options.min_x || options.max_x || options.min_y || options.max_y;
  if (has_radius && has_rect) {
    throw std::runtime_error("Use either rectangle crop or radius crop, not both");
  }

  return options;
}

Bounds makeBounds(const Options & options)
{
  Bounds bounds;
  bounds.min_z = options.min_z.value_or(bounds.min_z);
  bounds.max_z = options.max_z.value_or(bounds.max_z);

  if (options.radius) {
    const double expanded = *options.radius + options.margin;
    bounds.min_x = *options.center_x - expanded;
    bounds.max_x = *options.center_x + expanded;
    bounds.min_y = *options.center_y - expanded;
    bounds.max_y = *options.center_y + expanded;
    bounds.mode = "radius";
    return bounds;
  }

  const bool has_rect = options.min_x || options.max_x || options.min_y || options.max_y;
  if (has_rect) {
    bounds.min_x = options.min_x.value_or(bounds.min_x);
    bounds.max_x = options.max_x.value_or(bounds.max_x);
    bounds.min_y = options.min_y.value_or(bounds.min_y);
    bounds.max_y = options.max_y.value_or(bounds.max_y);
    if (options.min_x) {
      bounds.min_x -= options.margin;
    }
    if (options.max_x) {
      bounds.max_x += options.margin;
    }
    if (options.min_y) {
      bounds.min_y -= options.margin;
    }
    if (options.max_y) {
      bounds.max_y += options.margin;
    }
    bounds.mode = "rectangle";
  }

  return bounds;
}

bool insideBounds(const PointT & point, const Bounds & bounds)
{
  return point.x >= bounds.min_x && point.x <= bounds.max_x &&
    point.y >= bounds.min_y && point.y <= bounds.max_y &&
    point.z >= bounds.min_z && point.z <= bounds.max_z;
}

std::string jsonNumber(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6) << value;
  return ss.str();
}

void writeReport(
  const std::string & report_path,
  const Options & options,
  const Bounds & bounds,
  const std::string & output_path,
  std::size_t raw_points,
  std::size_t valid_points,
  std::size_t cropped_points,
  std::size_t output_points)
{
  std::ofstream report(report_path);
  report << "{\n"
         << "  \"input\": \"" << options.input << "\",\n"
         << "  \"output\": \"" << output_path << "\",\n"
         << "  \"crop_mode\": \"" << bounds.mode << "\",\n"
         << "  \"leaf_size_m\": " << jsonNumber(options.leaf) << ",\n"
         << "  \"margin_m\": " << jsonNumber(options.margin) << ",\n"
         << "  \"bounds\": {\n"
         << "    \"min_x\": " << jsonNumber(bounds.min_x) << ",\n"
         << "    \"max_x\": " << jsonNumber(bounds.max_x) << ",\n"
         << "    \"min_y\": " << jsonNumber(bounds.min_y) << ",\n"
         << "    \"max_y\": " << jsonNumber(bounds.max_y) << ",\n"
         << "    \"min_z\": " << jsonNumber(bounds.min_z) << ",\n"
         << "    \"max_z\": " << jsonNumber(bounds.max_z) << "\n"
         << "  },\n"
         << "  \"raw_points\": " << raw_points << ",\n"
         << "  \"valid_points\": " << valid_points << ",\n"
         << "  \"cropped_points\": " << cropped_points << ",\n"
         << "  \"output_points\": " << output_points << "\n"
         << "}\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parseArgs(argc, argv);
    const Bounds bounds = makeBounds(options);
    const std::string output_path = chooseOutputPath(options.output, options.force);
    const std::string report_path = output_path + ".report.json";

    CloudT::Ptr raw(new CloudT);
    if (pcl::io::loadPCDFile<PointT>(options.input, *raw) != 0) {
      throw std::runtime_error("Failed to load input PCD: " + options.input);
    }

    CloudT::Ptr valid(new CloudT);
    std::vector<int> kept_indices;
    pcl::removeNaNFromPointCloud(*raw, *valid, kept_indices);
    if (valid->empty()) {
      throw std::runtime_error("Input PCD has no valid points");
    }

    CloudT::Ptr cropped(new CloudT);
    cropped->reserve(valid->size());
    for (const auto & point : valid->points) {
      if (insideBounds(point, bounds)) {
        cropped->push_back(point);
      }
    }
    cropped->width = static_cast<std::uint32_t>(cropped->size());
    cropped->height = 1;
    cropped->is_dense = false;

    if (cropped->empty()) {
      throw std::runtime_error("Crop removed all points. Check crop bounds in the map frame.");
    }

    CloudT::Ptr output(new CloudT);
    if (options.leaf > 0.0) {
      pcl::VoxelGrid<PointT> voxel;
      voxel.setInputCloud(cropped);
      voxel.setLeafSize(
        static_cast<float>(options.leaf),
        static_cast<float>(options.leaf),
        static_cast<float>(options.leaf));
      voxel.filter(*output);
    } else {
      output = cropped;
    }

    if (output->empty()) {
      throw std::runtime_error("Downsampled output has no points");
    }

    if (pcl::io::savePCDFileBinaryCompressed(output_path, *output) != 0) {
      throw std::runtime_error("Failed to write output PCD: " + output_path);
    }

    writeReport(
      report_path,
      options,
      bounds,
      output_path,
      raw->size(),
      valid->size(),
      cropped->size(),
      output->size());

    std::cout << "Prepared local map:\n"
              << "  input: " << options.input << "\n"
              << "  output: " << output_path << "\n"
              << "  report: " << report_path << "\n"
              << "  crop_mode: " << bounds.mode << "\n"
              << "  raw_points: " << raw->size() << "\n"
              << "  valid_points: " << valid->size() << "\n"
              << "  cropped_points: " << cropped->size() << "\n"
              << "  output_points: " << output->size() << "\n"
              << "  leaf_size_m: " << options.leaf << "\n";
  } catch (const std::exception & ex) {
    std::cerr << "ERROR: " << ex.what() << "\n\n";
    printHelp();
    return 1;
  }

  return 0;
}
