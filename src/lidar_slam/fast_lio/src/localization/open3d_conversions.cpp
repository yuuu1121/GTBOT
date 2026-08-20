#include "fast_lio/localization/open3d_conversions.h"

namespace open3d_conversions
{

void open3dToRos(
    const open3d::geometry::PointCloud& pointcloud,
    sensor_msgs::msg::PointCloud2& ros_pc2,
    std::string frame_id)
{
    sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
    if (pointcloud.HasColors())
    {
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    }
    else
    {
        modifier.setPointCloud2FieldsByString(1, "xyz");
    }
    modifier.resize(pointcloud.points_.size());
    ros_pc2.header.frame_id = frame_id;

    sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");

    if (pointcloud.HasColors())
    {
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");

        for (size_t i = 0; i < pointcloud.points_.size();
             i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
        {
            const Eigen::Vector3d& point = pointcloud.points_[i];
            const Eigen::Vector3d& color = pointcloud.colors_[i];
            *ros_pc2_x = point(0);
            *ros_pc2_y = point(1);
            *ros_pc2_z = point(2);
            *ros_pc2_r = static_cast<uint8_t>(255 * color(0));
            *ros_pc2_g = static_cast<uint8_t>(255 * color(1));
            *ros_pc2_b = static_cast<uint8_t>(255 * color(2));
        }
    }
    else
    {
        for (size_t i = 0; i < pointcloud.points_.size();
             i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
        {
            const Eigen::Vector3d& point = pointcloud.points_[i];
            *ros_pc2_x = point(0);
            *ros_pc2_y = point(1);
            *ros_pc2_z = point(2);
        }
    }
}

void rosToOpen3d(
    const PointCloud2ConstPtr& ros_pc2,
    open3d::geometry::PointCloud& o3d_pc,
    bool skip_colors)
{
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");

    o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);

    if (ros_pc2->fields.size() == 3 || skip_colors)
    {
        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
             ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
        {
            o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        }
    }
    else
    {
        o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);

        // Check for RGB field
        bool has_rgb = false;
        bool has_intensity = false;
        for (const auto& field : ros_pc2->fields)
        {
            if (field.name == "rgb") has_rgb = true;
            if (field.name == "intensity") has_intensity = true;
        }

        if (has_rgb)
        {
            sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");

            for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
                 ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
            {
                o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
                o3d_pc.colors_.push_back(Eigen::Vector3d(
                    static_cast<int>(*ros_pc2_r) / 255.0,
                    static_cast<int>(*ros_pc2_g) / 255.0,
                    static_cast<int>(*ros_pc2_b) / 255.0));
            }
        }
        else if (has_intensity)
        {
            sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_i(*ros_pc2, "intensity");
            for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
                 ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
            {
                o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
                double intensity = std::min(1.0, std::max(0.0, static_cast<double>(*ros_pc2_i) / 255.0));
                o3d_pc.colors_.push_back(Eigen::Vector3d(intensity, intensity, intensity));
            }
        }
        else
        {
            // No color information, just copy xyz
            for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
                 ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
            {
                o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
            }
        }
    }
}

}  // namespace open3d_conversions
