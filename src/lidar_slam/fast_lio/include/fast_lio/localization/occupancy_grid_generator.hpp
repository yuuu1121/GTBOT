/**
 * @file occupancy_grid_generator.hpp
 * @brief Generate 2D OccupancyGrid from 3D point cloud with Z-axis filtering
 *
 * Converts Open3D point cloud to nav_msgs/OccupancyGrid by:
 * 1. Filtering points within z_min to z_max range
 * 2. Projecting remaining points onto XY plane
 * 3. Marking occupied cells in the grid
 */

#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>
#include <open3d/Open3D.h>
#include <string>
#include <memory>

namespace occupancy_grid
{

/**
 * @brief Parameters for OccupancyGrid generation
 */
struct OccupancyGridParams
{
    double resolution = 0.05;      ///< Grid cell size in meters
    double z_min = -0.5;           ///< Minimum Z height to include
    double z_max = 1.5;            ///< Maximum Z height to include
    std::string frame_id = "map";  ///< Frame ID for the grid
};

/**
 * @brief Generates 2D OccupancyGrid from 3D point cloud
 *
 * This class takes an Open3D point cloud and generates a nav_msgs::OccupancyGrid
 * suitable for visualization in RViz and use with navigation stacks.
 */
class OccupancyGridGenerator
{
public:
    /**
     * @brief Construct generator with parameters
     * @param params Grid generation parameters
     */
    explicit OccupancyGridGenerator(const OccupancyGridParams& params);

    /**
     * @brief Generate OccupancyGrid from Open3D point cloud
     * @param cloud Input 3D point cloud
     * @param stamp Timestamp for the output message
     * @return OccupancyGrid message, or nullptr if generation fails
     */
    nav_msgs::msg::OccupancyGrid::UniquePtr generate(
        const open3d::geometry::PointCloud& cloud,
        const rclcpp::Time& stamp) const;

    /**
     * @brief Get current parameters
     */
    const OccupancyGridParams& params() const { return params_; }

private:
    OccupancyGridParams params_;
};

}  // namespace occupancy_grid
