// ROS PointCloud2 (Ouster /ouster/points) -> PCL PointXYZI, placing a chosen
// scalar field into the intensity slot so the intensity-based clustering
// pipeline runs unchanged. Reads by field name via iterators, so it is robust
// to field offset/order. The scalar field is selectable: "reflectivity" (uint16,
// distance-corrected surface reflectivity) or "intensity" (float32, raw signal).
#ifndef OUSTER_CLUSTER__OUSTER_CONVERT_HPP_
#define OUSTER_CLUSTER__OUSTER_CONVERT_HPP_

#include <algorithm>
#include <cmath>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "ouster_cluster/cluster_logic.hpp"  // ouster_cluster::Cloud
#include "ouster_cluster/elevation_filter.hpp"  // ElevationFilterParams

namespace ouster_cluster
{

/// The PointField datatype of the named field, or -1 if absent.
inline int fieldDatatype(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & f : msg.fields) {
    if (f.name == name) {return f.datatype;}
  }
  return -1;
}

/// True if `msg` has a field named `name`.
inline bool hasField(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  return fieldDatatype(msg, name) >= 0;
}

/// Convert an Ouster PointCloud2 to PCL PointXYZI, putting `scalar_field` into
/// the intensity slot. `scalar_field` is read with the iterator type matching
/// its declared PointField datatype (uint16 for reflectivity, float32 for
/// intensity), so either Ouster field works. Returns an empty cloud if x/y/z or
/// the requested scalar field is absent.
inline Cloud::Ptr fromOusterMsg(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::string & scalar_field = "reflectivity")
{
  auto out = std::make_shared<Cloud>();
  const int dt = fieldDatatype(msg, scalar_field);
  if (!hasField(msg, "x") || !hasField(msg, "y") || !hasField(msg, "z") || dt < 0) {
    return out;  // empty — caller warns
  }
  const std::size_t n = static_cast<std::size_t>(msg.width) * msg.height;
  if (msg.point_step == 0 ||
    static_cast<std::size_t>(msg.point_step) * n > msg.data.size())
  {
    return out;  // truncated/malformed buffer — refuse instead of reading past it
  }
  out->reserve(n);
  sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");

  // Read the scalar with the iterator type that matches its declared datatype.
  // (reflectivity is UINT16, intensity is FLOAT32 on Ouster.)
  if (dt == sensor_msgs::msg::PointField::FLOAT32) {
    sensor_msgs::PointCloud2ConstIterator<float> is(msg, scalar_field);
    for (std::size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz, ++is) {
      pcl::PointXYZI p;
      p.x = *ix; p.y = *iy; p.z = *iz;
      p.intensity = *is;
      out->push_back(p);
    }
  } else if (dt == sensor_msgs::msg::PointField::UINT16) {
    sensor_msgs::PointCloud2ConstIterator<uint16_t> is(msg, scalar_field);
    for (std::size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz, ++is) {
      pcl::PointXYZI p;
      p.x = *ix; p.y = *iy; p.z = *iz;
      p.intensity = static_cast<float>(*is);
      out->push_back(p);
    }
  } else {
    return out;  // unsupported datatype -> empty, caller warns
  }
  out->width = out->size();
  out->height = 1;
  out->is_dense = false;  // may contain NaN xyz; node runs removeNaN next
  return out;
}

/// Fused overload: convert + NaN drop + elevation band cut in ONE pass over
/// the message, replacing convert -> removeNaN -> filterElevation (three
/// full-cloud passes and two extra allocations on ~130k points per frame).
/// Band semantics match filterElevation (z vs r_xy * tan(bound), inclusive);
/// the returned cloud is dense (no NaN survives).
inline Cloud::Ptr fromOusterMsg(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::string & scalar_field,
  const ElevationFilterParams & elev)
{
  auto out = std::make_shared<Cloud>();
  const int dt = fieldDatatype(msg, scalar_field);
  if (!hasField(msg, "x") || !hasField(msg, "y") || !hasField(msg, "z") || dt < 0) {
    return out;  // empty — caller warns
  }
  const bool band = elev.enable;
  const float tan_min = static_cast<float>(std::tan(elev.min_deg * M_PI / 180.0));
  const float tan_max = static_cast<float>(std::tan(elev.max_deg * M_PI / 180.0));
  const auto keep = [band, tan_min, tan_max](float x, float y, float z) {
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {return false;}
      if (!band) {return true;}
      const float r_xy = std::hypot(x, y);
      return z >= r_xy * tan_min && z <= r_xy * tan_max;
    };
  const std::size_t n = static_cast<std::size_t>(msg.width) * msg.height;
  if (msg.point_step == 0 ||
    static_cast<std::size_t>(msg.point_step) * n > msg.data.size())
  {
    return out;  // truncated/malformed buffer — refuse instead of reading past it
  }
  out->reserve(n);
  sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");

  if (dt == sensor_msgs::msg::PointField::FLOAT32) {
    sensor_msgs::PointCloud2ConstIterator<float> is(msg, scalar_field);
    for (std::size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz, ++is) {
      if (!keep(*ix, *iy, *iz)) {continue;}
      pcl::PointXYZI p;
      p.x = *ix; p.y = *iy; p.z = *iz;
      p.intensity = *is;
      out->push_back(p);
    }
  } else if (dt == sensor_msgs::msg::PointField::UINT16) {
    sensor_msgs::PointCloud2ConstIterator<uint16_t> is(msg, scalar_field);
    for (std::size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz, ++is) {
      if (!keep(*ix, *iy, *iz)) {continue;}
      pcl::PointXYZI p;
      p.x = *ix; p.y = *iy; p.z = *iz;
      p.intensity = static_cast<float>(*is);
      out->push_back(p);
    }
  } else {
    return out;  // unsupported datatype -> empty, caller warns
  }
  out->width = out->size();
  out->height = 1;
  out->is_dense = true;  // NaN dropped in the keep() gate
  return out;
}

}  // namespace ouster_cluster

#endif  // OUSTER_CLUSTER__OUSTER_CONVERT_HPP_
