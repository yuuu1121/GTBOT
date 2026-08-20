/**
 * @file occupancy_grid_generator.cpp
 * @brief Implementation of OccupancyGrid generation from 3D point cloud
 */

#include "fast_lio/localization/occupancy_grid_generator.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace occupancy_grid
{

OccupancyGridGenerator::OccupancyGridGenerator(const OccupancyGridParams& params)
    : params_(params)
{
}

nav_msgs::msg::OccupancyGrid::UniquePtr OccupancyGridGenerator::generate(
    const open3d::geometry::PointCloud& cloud,
    const rclcpp::Time& stamp) const
{
    if (cloud.IsEmpty())
    {
        return nullptr;
    }

    // Step 1: Filter points by Z range and find XY bounds
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();

    std::vector<Eigen::Vector2d> xy_points;
    xy_points.reserve(cloud.points_.size() / 4);  // Rough estimate

    for (const auto& pt : cloud.points_)
    {
        if (pt.z() >= params_.z_min && pt.z() <= params_.z_max)
        {
            xy_points.emplace_back(pt.x(), pt.y());
            x_min = std::min(x_min, pt.x());
            x_max = std::max(x_max, pt.x());
            y_min = std::min(y_min, pt.y());
            y_max = std::max(y_max, pt.y());
        }
    }

    if (xy_points.empty())
    {
        return nullptr;
    }

    // Step 2: Add margin to bounds
    const double margin = params_.resolution * 2;
    x_min -= margin;
    y_min -= margin;
    x_max += margin;
    y_max += margin;

    // Step 3: Calculate grid dimensions
    const uint32_t width = static_cast<uint32_t>(
        std::ceil((x_max - x_min) / params_.resolution));
    const uint32_t height = static_cast<uint32_t>(
        std::ceil((y_max - y_min) / params_.resolution));

    // Sanity check for extremely large grids
    const uint32_t max_cells = 10000 * 10000;  // 100M cells max
    if (static_cast<uint64_t>(width) * height > max_cells)
    {
        return nullptr;
    }

    // Step 4: Create OccupancyGrid message
    auto grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid->header.stamp = stamp;
    grid->header.frame_id = params_.frame_id;

    grid->info.resolution = static_cast<float>(params_.resolution);
    grid->info.width = width;
    grid->info.height = height;
    grid->info.origin.position.x = x_min;
    grid->info.origin.position.y = y_min;
    grid->info.origin.position.z = 0.0;
    grid->info.origin.orientation.w = 1.0;

    // Initialize all cells as unknown (-1)
    // Then mark cells with points as occupied (100), leaving rest as free (0)
    // For a point cloud map, we mark: occupied=100, free=0 (no unknown)
    grid->data.resize(width * height, 0);  // Initialize as free

    // Step 5: Project points onto grid
    for (const auto& pt : xy_points)
    {
        int cell_x = static_cast<int>((pt.x() - x_min) / params_.resolution);
        int cell_y = static_cast<int>((pt.y() - y_min) / params_.resolution);

        // Bounds check
        if (cell_x >= 0 && cell_x < static_cast<int>(width) &&
            cell_y >= 0 && cell_y < static_cast<int>(height))
        {
            size_t idx = cell_y * width + cell_x;
            grid->data[idx] = 100;  // Occupied
        }
    }

    return grid;
}

}  // namespace occupancy_grid
