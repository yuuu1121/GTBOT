/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <map>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_slam/node.h"
#include "cartographer_slam/node_options.h"
#include "cartographer_slam/ros_log_sink.h"
#include "cartographer_slam/ros_map.h"
#include "gflags/gflags.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");
DEFINE_double(
    map_resolution, 0.05,
    "Resolution of the 2D map saved alongside the state file.");

namespace cartographer_ros {
namespace {

void Run() {
  rclcpp::Node::SharedPtr cartographer_node = rclcpp::Node::make_shared("cartographer_node");
  constexpr double kTfBufferCacheTimeInSeconds = 10.;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
        cartographer_node->get_clock(),
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
        cartographer_node);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  auto map_builder =
    cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  auto node = std::make_shared<cartographer_ros::Node>(
    node_options, std::move(map_builder), tf_buffer, cartographer_node,
    FLAGS_collect_metrics);
  if (!FLAGS_load_state_filename.empty()) {
    node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  rclcpp::spin(cartographer_node);

  node->FinishAllTrajectories();
  node->RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                        false /* include_unfinished_submaps - false prevents double rendering */);

    // Also save 2D map (pgm + yaml) alongside the pbstream file
    LOG(INFO) << "Generating 2D map from saved state...";
    try {
      ::cartographer::io::ProtoStreamReader reader(FLAGS_save_state_filename);
      ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

      std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
          submap_slices;
      ::cartographer::mapping::ValueConversionTables conversion_tables;
      ::cartographer::io::DeserializeAndFillSubmapSlices(
          &deserializer, &submap_slices, &conversion_tables);

      if (!submap_slices.empty()) {
        auto result =
            ::cartographer::io::PaintSubmapSlices(submap_slices, FLAGS_map_resolution);

        // Extract filestem from save_state_filename (remove .pbstream extension)
        std::string map_filestem = FLAGS_save_state_filename;
        const std::string pbstream_ext = ".pbstream";
        if (map_filestem.size() > pbstream_ext.size() &&
            map_filestem.substr(map_filestem.size() - pbstream_ext.size()) == pbstream_ext) {
          map_filestem = map_filestem.substr(0, map_filestem.size() - pbstream_ext.size());
        }

        ::cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");
        ::cartographer::io::Image image(std::move(result.surface));
        WritePgm(image, FLAGS_map_resolution, &pgm_writer);

        const Eigen::Vector2d origin(
            -result.origin.x() * FLAGS_map_resolution,
            (result.origin.y() - image.height()) * FLAGS_map_resolution);

        ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
        WriteYaml(FLAGS_map_resolution, origin, pgm_writer.GetFilename(), &yaml_writer);

        LOG(INFO) << "2D map saved to " << map_filestem << ".pgm and " << map_filestem << ".yaml";
      } else {
        LOG(WARNING) << "No submaps found, skipping 2D map generation.";
      }
    } catch (const std::exception& e) {
      LOG(ERROR) << "Failed to generate 2D map: " << e.what();
    }
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::rclcpp::shutdown();
}
