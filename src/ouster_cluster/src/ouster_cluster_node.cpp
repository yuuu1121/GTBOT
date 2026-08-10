// ouster_cluster_node: subscribe Ouster /ouster/points, build a BEV density
// map, extract line segments, classify plate candidates, and publish
// BEV-tracked wireframe boxes as a MarkerArray.
#include <chrono>
#include <deque>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include "ouster_cluster/cluster_logic.hpp"
#include "ouster_cluster/bev_tracker.hpp"
#include "ouster_cluster/plane_detect.hpp"
#include "ouster_cluster/bev_density.hpp"
#include "ouster_cluster/bev_lines.hpp"
#include "ouster_cluster/ouster_convert.hpp"
#include "ouster_cluster/elevation_filter.hpp"
#ifdef OUSTER_CLUSTER_HAS_CUDA
#include "ouster_cluster/gpu_frontend.hpp"
#endif

namespace ouster_cluster
{

class OusterClusterNode : public rclcpp::Node
{
public:
  OusterClusterNode()
  : Node("ouster_cluster_node")
  {
    bev_prm_.match_distance = declare_parameter("track_match_distance", bev_prm_.match_distance);
    bev_prm_.pos_alpha = declare_parameter("track_pos_alpha", bev_prm_.pos_alpha);
    bev_prm_.vel_alpha = declare_parameter("track_vel_alpha", bev_prm_.vel_alpha);
    bev_prm_.yaw_alpha = declare_parameter("track_yaw_alpha", bev_prm_.yaw_alpha);
    bev_prm_.confirm_frames = declare_parameter("track_confirm_frames", bev_prm_.confirm_frames);
    bev_prm_.max_missed = declare_parameter("track_max_missed", bev_prm_.max_missed);
    bev_prm_.max_dt = declare_parameter("track_max_dt", bev_prm_.max_dt);
    input_topic_ = declare_parameter("input_topic", std::string("/ouster/points"));
    scalar_field_ = declare_parameter("scalar_field", std::string("reflectivity"));
    elev_prm_.enable = declare_parameter("elev_filter_enable", elev_prm_.enable);
    elev_prm_.min_deg = declare_parameter("elev_min_deg", elev_prm_.min_deg);
    elev_prm_.max_deg = declare_parameter("elev_max_deg", elev_prm_.max_deg);
    pre_.max_range = declare_parameter("preprocess_max_range", pre_.max_range);
    pre_.z_min = declare_parameter("preprocess_z_min", pre_.z_min);
    pre_.z_max = declare_parameter("preprocess_z_max", pre_.z_max);
    pre_.voxel_leaf = declare_parameter("preprocess_voxel_leaf", pre_.voxel_leaf);
    pre_.min_intensity = declare_parameter("preprocess_min_intensity", pre_.min_intensity);
    // GPU front-end (conv+cut / crop / BEV density fused on CUDA). Falls back
    // to the CPU path per-frame when unavailable or unsupported (voxel on).
    use_gpu_ = declare_parameter("use_gpu", true);
#ifdef OUSTER_CLUSTER_HAS_CUDA
    if (use_gpu_) {
      if (GpuFrontend::available()) {
        gpu_ = std::make_unique<GpuFrontend>();
        RCLCPP_INFO(get_logger(), "GPU front-end enabled (CUDA device found)");
      } else {
        RCLCPP_WARN(get_logger(), "use_gpu requested but no CUDA device; using CPU path");
      }
    }
#else
    if (use_gpu_) {
      RCLCPP_INFO(get_logger(), "built without CUDA; using CPU path");
    }
#endif
    bev_.cell_size = declare_parameter("bev_density_cell_size", bev_.cell_size);
    bev_.min_density = declare_parameter("bev_density_min", bev_.min_density);
    bev_.max_density = declare_parameter("bev_density_max", bev_.max_density);
    bev_.z_bin = declare_parameter("bev_density_z_bin", bev_.z_bin);
    bev_.min_pts_per_bin = declare_parameter("bev_density_min_pts_per_bin", bev_.min_pts_per_bin);
    bev_.min_z_bins = declare_parameter("bev_density_min_z_bins", bev_.min_z_bins);
    bev_.dual_grid = declare_parameter("bev_density_dual_grid", bev_.dual_grid);
    bev_.drop_lonely = declare_parameter("bev_density_drop_lonely", bev_.drop_lonely);
    line_prm_.min_cells = declare_parameter("bev_line_min_cells", line_prm_.min_cells);
    line_prm_.max_width = declare_parameter("bev_line_max_width", line_prm_.max_width);
    line_prm_.min_length = declare_parameter("bev_line_min_length", line_prm_.min_length);
    line_prm_.max_length = declare_parameter("bev_line_max_length", line_prm_.max_length);
    line_prm_.z_gap = declare_parameter("bev_line_z_gap", line_prm_.z_gap);
    line_prm_.z_med_gap = declare_parameter("bev_line_z_med_gap", line_prm_.z_med_gap);
    line_prm_.diag_flank_gate =
      declare_parameter("bev_line_diag_flank_gate", line_prm_.diag_flank_gate);
    line_prm_.touch_connect =
      declare_parameter("bev_line_touch_connect", line_prm_.touch_connect);
    line_prm_.touch_gap = declare_parameter("bev_line_touch_gap", line_prm_.touch_gap);
    line_prm_.touch_xy_gap =
      declare_parameter("bev_line_touch_xy_gap", line_prm_.touch_xy_gap);
    line_prm_.touch_min_matched = static_cast<int>(
      declare_parameter("bev_line_touch_min_matched", line_prm_.touch_min_matched));
    line_prm_.touch_min_ratio =
      declare_parameter("bev_line_touch_min_ratio", line_prm_.touch_min_ratio);
    line_prm_.top_tol = declare_parameter("bev_line_top_tol", line_prm_.top_tol);
    line_prm_.top_step = declare_parameter("bev_line_top_step", line_prm_.top_step);
    line_prm_.max_segments = declare_parameter("bev_line_max_segments", line_prm_.max_segments);
    refine_prm_.enable = declare_parameter("bev_refine_enable", refine_prm_.enable);
    refine_prm_.band = declare_parameter("bev_refine_band", refine_prm_.band);
    refine_prm_.mad_k = declare_parameter("bev_refine_mad_k", refine_prm_.mad_k);
    refine_prm_.end_percentile = declare_parameter("bev_refine_end_percentile", refine_prm_.end_percentile);
    refine_prm_.min_points = declare_parameter("bev_refine_min_points", refine_prm_.min_points);
    bev_plate_prm_.width = declare_parameter("bev_plate_width", bev_plate_prm_.width);
    bev_plate_prm_.height = declare_parameter("bev_plate_height", bev_plate_prm_.height);
    bev_plate_prm_.tol_width =
      declare_parameter("bev_plate_tol_width", bev_plate_prm_.tol_width);
    bev_plate_prm_.tol_height =
      declare_parameter("bev_plate_tol_height", bev_plate_prm_.tol_height);
    bev_plate_prm_.thickness_max =
      declare_parameter("bev_plate_thickness_max", bev_plate_prm_.thickness_max);
    seg_prm_.match_distance = declare_parameter("bev_seg_match_distance", seg_prm_.match_distance);
    seg_prm_.confirm_frames = declare_parameter("bev_seg_confirm_frames", seg_prm_.confirm_frames);
    seg_prm_.max_missed = declare_parameter("bev_seg_max_missed", seg_prm_.max_missed);
    refl_prm_.enable = declare_parameter("bev_refl_enable", refl_prm_.enable);
    refl_prm_.band = declare_parameter("bev_refl_band", refl_prm_.band);
    refl_prm_.spread_max = declare_parameter("bev_refl_spread_max", refl_prm_.spread_max);
    refl_prm_.min_median = declare_parameter("bev_refl_min", refl_prm_.min_median);
    refl_prm_.max_median = declare_parameter("bev_refl_max", refl_prm_.max_median);
    refl_prm_.min_points = declare_parameter("bev_refl_min_points", refl_prm_.min_points);
    recover_enable_ = declare_parameter("plate_recover_enable", recover_enable_);
    recover_prm_.xy_half = declare_parameter("plate_recover_xy_half", recover_prm_.xy_half);
    recover_prm_.z_margin_below =
      declare_parameter("plate_recover_z_margin_below", recover_prm_.z_margin_below);
    recover_prm_.z_margin_above =
      declare_parameter("plate_recover_z_margin_above", recover_prm_.z_margin_above);
    recover_prm_.tolerance =
      declare_parameter("plate_recover_tolerance", recover_prm_.tolerance);
    recover_prm_.min_points = static_cast<int>(
      declare_parameter("plate_recover_min_points", recover_prm_.min_points));
    recover_prm_.maha_threshold =
      declare_parameter("plate_recover_maha_threshold", recover_prm_.maha_threshold);
    recover_accum_frames_ = static_cast<int>(
      declare_parameter("plate_recover_accum_frames", recover_accum_frames_));
    pp_.thickness_max = declare_parameter("plate_thickness_max", pp_.thickness_max);
    pp_.width = declare_parameter("plate_width", pp_.width);
    pp_.height = declare_parameter("plate_height", pp_.height);
    pp_.size_tol = declare_parameter("plate_size_tol", pp_.size_tol);
    pp_.min_points = declare_parameter("plate_min_points", pp_.min_points);
    // per-stage latency log (throttled, ~2s) — for hunting CPU hotspots
    timing_log_ = declare_parameter("timing_log", timing_log_);

    param_cb_ = add_on_set_parameters_callback(
      std::bind(&OusterClusterNode::onSetParams, this, std::placeholders::_1));

    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&OusterClusterNode::onCloud, this, std::placeholders::_1));
    pub_boxes_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/ouster_cluster/boxes", 10);
    // BEV density: each point's intensity = point count of its XY cell. High
    // intensity marks XY cells where many points stack in z (upright structures).
    pub_bev_density_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ouster_cluster/bev_density", qos);
    // Raw cloud right after the elevation band cut — for eyeballing the cut in
    // RViz. Only serialized when someone subscribes.
    pub_points_filtered_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ouster_cluster/points_filtered", qos);
    // BEV line segments (vertical-structure candidates) as LINE_LIST markers.
    pub_bev_lines_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/ouster_cluster/bev_lines", 10);

    RCLCPP_INFO(
      get_logger(), "ouster_cluster_node up. input=%s", input_topic_.c_str());
  }

private:
  rcl_interfaces::msg::SetParametersResult onSetParams(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      const auto & n = p.get_name();
      if (n == "track_match_distance") {bev_prm_.match_distance = p.as_double();}
      else if (n == "track_pos_alpha") {bev_prm_.pos_alpha = p.as_double();}
      else if (n == "track_vel_alpha") {bev_prm_.vel_alpha = p.as_double();}
      else if (n == "track_yaw_alpha") {bev_prm_.yaw_alpha = p.as_double();}
      else if (n == "track_confirm_frames") {bev_prm_.confirm_frames = static_cast<int>(p.as_int());}
      else if (n == "track_max_missed") {bev_prm_.max_missed = static_cast<int>(p.as_int());}
      else if (n == "track_max_dt") {bev_prm_.max_dt = p.as_double();}
      else if (n == "scalar_field") {scalar_field_ = p.as_string();}
      else if (n == "timing_log") {timing_log_ = p.as_bool();}
      else if (n == "elev_filter_enable") {elev_prm_.enable = p.as_bool();}
      else if (n == "elev_min_deg") {elev_prm_.min_deg = p.as_double();}
      else if (n == "elev_max_deg") {elev_prm_.max_deg = p.as_double();}
      else if (n == "preprocess_max_range") {pre_.max_range = p.as_double();}
      else if (n == "preprocess_z_min") {pre_.z_min = p.as_double();}
      else if (n == "preprocess_z_max") {pre_.z_max = p.as_double();}
      else if (n == "preprocess_voxel_leaf") {pre_.voxel_leaf = p.as_double();}
      else if (n == "preprocess_min_intensity") {pre_.min_intensity = p.as_double();}
      else if (n == "use_gpu") {
        use_gpu_ = p.as_bool();
#ifdef OUSTER_CLUSTER_HAS_CUDA
        // a node started with use_gpu=false never built the front-end; create
        // it on the first live enable so the toggle works both ways
        if (use_gpu_ && !gpu_ && GpuFrontend::available()) {
          gpu_ = std::make_unique<GpuFrontend>();
          RCLCPP_INFO(get_logger(), "GPU front-end enabled (live)");
        }
#endif
      }
      else if (n == "bev_density_cell_size") {bev_.cell_size = p.as_double();}
      else if (n == "bev_density_min") {bev_.min_density = p.as_double();}
      else if (n == "bev_density_max") {bev_.max_density = p.as_double();}
      else if (n == "bev_density_z_bin") {bev_.z_bin = p.as_double();}
      else if (n == "bev_density_min_pts_per_bin") {bev_.min_pts_per_bin = static_cast<int>(p.as_int());}
      else if (n == "bev_density_min_z_bins") {bev_.min_z_bins = static_cast<int>(p.as_int());}
      else if (n == "bev_density_dual_grid") {bev_.dual_grid = p.as_bool();}
      else if (n == "bev_density_drop_lonely") {bev_.drop_lonely = p.as_bool();}
      else if (n == "bev_line_min_cells") {line_prm_.min_cells = static_cast<int>(p.as_int());}
      else if (n == "bev_line_max_width") {line_prm_.max_width = p.as_double();}
      else if (n == "bev_line_min_length") {line_prm_.min_length = p.as_double();}
      else if (n == "bev_line_max_length") {line_prm_.max_length = p.as_double();}
      else if (n == "bev_line_z_gap") {line_prm_.z_gap = p.as_double();}
      else if (n == "bev_line_z_med_gap") {line_prm_.z_med_gap = p.as_double();}
      else if (n == "bev_line_diag_flank_gate") {line_prm_.diag_flank_gate = p.as_bool();}
      else if (n == "bev_line_touch_connect") {line_prm_.touch_connect = p.as_bool();}
      else if (n == "bev_line_touch_gap") {line_prm_.touch_gap = p.as_double();}
      else if (n == "bev_line_touch_xy_gap") {line_prm_.touch_xy_gap = p.as_double();}
      else if (n == "bev_line_touch_min_matched") {
        line_prm_.touch_min_matched = static_cast<int>(p.as_int());
      }
      else if (n == "bev_line_touch_min_ratio") {line_prm_.touch_min_ratio = p.as_double();}
      else if (n == "bev_line_top_tol") {line_prm_.top_tol = p.as_double();}
      else if (n == "bev_line_top_step") {line_prm_.top_step = p.as_double();}
      else if (n == "bev_line_max_segments") {line_prm_.max_segments = static_cast<int>(p.as_int());}
      else if (n == "bev_refine_enable") {refine_prm_.enable = p.as_bool();}
      else if (n == "bev_refine_band") {refine_prm_.band = p.as_double();}
      else if (n == "bev_refine_mad_k") {refine_prm_.mad_k = p.as_double();}
      else if (n == "bev_refine_end_percentile") {refine_prm_.end_percentile = p.as_double();}
      else if (n == "bev_refine_min_points") {refine_prm_.min_points = static_cast<int>(p.as_int());}
      else if (n == "bev_plate_width") {bev_plate_prm_.width = p.as_double();}
      else if (n == "bev_plate_height") {bev_plate_prm_.height = p.as_double();}
      else if (n == "bev_plate_tol_width") {bev_plate_prm_.tol_width = p.as_double();}
      else if (n == "bev_plate_tol_height") {bev_plate_prm_.tol_height = p.as_double();}
      else if (n == "bev_plate_thickness_max") {bev_plate_prm_.thickness_max = p.as_double();}
      else if (n == "bev_seg_match_distance") {seg_prm_.match_distance = p.as_double();}
      else if (n == "bev_seg_confirm_frames") {seg_prm_.confirm_frames = static_cast<int>(p.as_int());}
      else if (n == "bev_seg_max_missed") {seg_prm_.max_missed = static_cast<int>(p.as_int());}
      else if (n == "bev_refl_enable") {refl_prm_.enable = p.as_bool();}
      else if (n == "bev_refl_band") {refl_prm_.band = p.as_double();}
      else if (n == "bev_refl_spread_max") {refl_prm_.spread_max = p.as_double();}
      else if (n == "bev_refl_min") {refl_prm_.min_median = p.as_double();}
      else if (n == "bev_refl_max") {refl_prm_.max_median = p.as_double();}
      else if (n == "bev_refl_min_points") {refl_prm_.min_points = static_cast<int>(p.as_int());}
      else if (n == "plate_recover_enable") {recover_enable_ = p.as_bool();}
      else if (n == "plate_recover_xy_half") {recover_prm_.xy_half = p.as_double();}
      else if (n == "plate_recover_z_margin_below") {recover_prm_.z_margin_below = p.as_double();}
      else if (n == "plate_recover_z_margin_above") {recover_prm_.z_margin_above = p.as_double();}
      else if (n == "plate_recover_tolerance") {recover_prm_.tolerance = p.as_double();}
      else if (n == "plate_recover_min_points") {
        recover_prm_.min_points = static_cast<int>(p.as_int());
      }
      else if (n == "plate_recover_maha_threshold") {
        recover_prm_.maha_threshold = p.as_double();
      }
      else if (n == "plate_recover_accum_frames") {
        recover_accum_frames_ = static_cast<int>(p.as_int());
      }
      else if (n == "plate_thickness_max") {pp_.thickness_max = p.as_double();}
      else if (n == "plate_width") {pp_.width = p.as_double();}
      else if (n == "plate_height") {pp_.height = p.as_double();}
      else if (n == "plate_size_tol") {pp_.size_tol = p.as_double();}
      else if (n == "plate_min_points") {pp_.min_points = static_cast<int>(p.as_int());}
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Per-stage latency stamps (logged throttled at the end when timing_log).
    using SClock = std::chrono::steady_clock;
    const auto t0 = SClock::now();

    // Front-end: convert + NaN/elevation cut -> range/z crop -> BEV density.
    // GPU path fuses all three on CUDA (one pass over the raw buffer, results
    // point-order-identical to the CPU path); CPU path runs them in sequence.
    // In GPU mode the whole front-end lands in the "conv+cut" timing slot.
    Cloud::Ptr cloud;
    Cloud::Ptr density;
    std::size_t n_elev = 0;
    auto t_conv = t0, t_pubf = t0, t_prep = t0;
    bool gpu_done = false;
#ifdef OUSTER_CLUSTER_HAS_CUDA
    if (use_gpu_ && gpu_ && pre_.voxel_leaf > 0.0) {
      // voxel downsampling is not ported to the GPU; say so instead of
      // silently running the CPU path under an "enabled" startup log
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "voxel_leaf > 0: GPU front-end skipped, using CPU path");
    }
    if (use_gpu_ && gpu_ && pre_.voxel_leaf <= 0.0) {
      const bool want_conv = pub_points_filtered_->get_subscription_count() > 0;
      GpuFrontendResult r;
      if (gpu_->process(*msg, scalar_field_, elev_prm_, pre_, bev_, want_conv, r)) {
        n_elev = r.n_conv;
        if (n_elev == 0) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "empty cloud after conversion (missing x/y/z or %s field?)",
            scalar_field_.c_str());
          return;
        }
        if (want_conv && r.conv) {
          sensor_msgs::msg::PointCloud2 ros_filtered;
          pcl::toROSMsg(*r.conv, ros_filtered);
          ros_filtered.header = msg->header;
          pub_points_filtered_->publish(ros_filtered);
        }
        cloud = r.prep;
        density = r.density;
        gpu_done = true;
        t_conv = t_pubf = t_prep = SClock::now();
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "GPU front-end failed; using CPU path");
      }
    }
#endif
    if (!gpu_done) {
      // Ouster: pull the chosen scalar into the intensity slot, drop NaN and
      // cut the ray-elevation band (firmware cannot mask vertical beams) — all
      // fused into ONE pass over the ~130k-point message (was three passes).
      cloud = fromOusterMsg(*msg, scalar_field_, elev_prm_);
      t_conv = SClock::now();
      n_elev = cloud->size();
      if (cloud->empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "empty cloud after conversion (missing x/y/z or %s field?)",
          scalar_field_.c_str());
        return;
      }
      if (pub_points_filtered_->get_subscription_count() > 0) {
        sensor_msgs::msg::PointCloud2 ros_filtered;
        pcl::toROSMsg(*cloud, ros_filtered);
        ros_filtered.header = msg->header;
        pub_points_filtered_->publish(ros_filtered);
      }
      t_pubf = SClock::now();

      // Cheap front-of-pipeline cull so the heavy stages keep up in real time:
      // drop points beyond max_range, then voxel-downsample.
      cloud = preprocessCloud(cloud, pre_);
      t_prep = SClock::now();
    }
    if (cloud->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "empty cloud after preprocess (max_range too small?)");
      return;
    }

    // BEV density map: intensity = per-cell count x dist^2 (distance-invariant),
    // band-passed to [min, max] density. Published from the filtered (3m +
    // z-band) cloud, independent of the detection path below.
    if (!gpu_done) {
      density = computeBevDensity(cloud, bev_);
    }
    const auto t_dens = SClock::now();
    auto t_pubd = t_dens, t_lines = t_dens, t_rec = t_dens;
    std::size_t n_segs = 0;
    // recover sub-stage times (accumulate copy / per-seg recover / plate fit)
    double rec_acc_ms = 0.0, rec_seg_ms = 0.0, rec_fit_ms = 0.0;
    int rec_nseg = 0;

    // Keep the last N DENSITY frames for the recovery stage: the density map
    // has already dropped the floor/stand (z-bin verticality gate), so the
    // recovery cluster cannot bridge into them — unlike the raw preprocessed
    // cloud, where the stand rides along and fuses the plate into a tall blob.
    // At long range one frame's rings are sparse; successive frames interleave
    // and their union fills the gaps. Only recovery uses the accumulation.
    recover_buf_.push_back(density);
    while (static_cast<int>(recover_buf_.size()) > std::max(recover_accum_frames_, 1)) {
      recover_buf_.pop_front();
    }
    {
      sensor_msgs::msg::PointCloud2 ros_density;
      pcl::toROSMsg(*density, ros_density);
      ros_density.header = msg->header;
      pub_bev_density_->publish(ros_density);
      t_pubd = SClock::now();

      // Line segments over the surviving cells: planar structures seen from
      // above are lines. Cheap (runs on deduped cells), so always on. The
      // temporal filter publishes a segment only after N consecutive frames
      // and holds it through short dropouts (kills flickering noise segments).
      // coarse cell segments -> point-level refit (kills cell-grid jitter and
      // stray-point influence) -> temporal confirm/hold filter
      line_prm_.cell_size = bev_.cell_size;
      seg_filter_.setParams(seg_prm_);
      auto segs = refineBevSegments(
        extractBevLineSegments(density, line_prm_), density, refine_prm_);
      // One-material check against the preprocessed cloud (its intensity still
      // carries the scalar_field value; the density cloud's does not).
      if (refl_prm_.enable) {
        std::vector<BevSegment> kept;
        kept.reserve(segs.size());
        for (const auto & s : segs) {
          if (passesReflectivityGate(s, cloud, refl_prm_)) {kept.push_back(s);}
        }
        segs = std::move(kept);
      }
      const auto live_segs = seg_filter_.update(segs);
      publishBevLines(live_segs, msg->header);
      t_lines = SClock::now();
      n_segs = live_segs.size();

      // Final precision stage: for each GREEN segment, cluster the DENSITY
      // survivors around it (seed-grown, z-cropped) and shape-check the
      // cluster; confirmed plates go through the EMA tracker to the boxes
      // topic. Recovers the full plate when the density gates shortened the
      // segment, while the floor/stand stay out (they were dropped from the
      // density map).
      if (recover_enable_) {
        // accumulate the buffered density frames once; recover every segment
        const Cloud::Ptr accum = recover_accum_frames_ > 1
          ? accumulateClouds(std::vector<Cloud::Ptr>(recover_buf_.begin(), recover_buf_.end()))
          : density;
        const auto t_acc = SClock::now();
        std::vector<BevMeasurement> meas;
        for (const auto & s : live_segs) {
          if (!isPlateSegment(s, bev_plate_prm_)) {continue;}
          ++rec_nseg;
          const double cx = 0.5 * (s.x0 + s.x1);
          const double cy = 0.5 * (s.y0 + s.y1);
          const auto tr0 = SClock::now();
          auto rec = recoverPlateCluster(accum, cx, cy, s.z_min, s.z_max, recover_prm_);
          const auto tr1 = SClock::now();
          rec_seg_ms += std::chrono::duration<double, std::milli>(tr1 - tr0).count();
          if (rec->empty()) {continue;}
          pcl::PointIndices all;
          all.indices.resize(rec->size());
          std::iota(all.indices.begin(), all.indices.end(), 0);
          const bool ok = fitRectangularPlate(*rec, all, pp_).is_plate;
          rec_fit_ms += std::chrono::duration<double, std::milli>(
            SClock::now() - tr1).count();
          if (!ok) {continue;}
          meas.push_back(clusterToBevMeasurement(*rec, all));
        }
        rec_acc_ms = std::chrono::duration<double, std::milli>(t_acc - t_lines).count();
        const double dt = computeDt(msg->header);
        bev_tracker_.setParams(bev_prm_);
        publishTrackBoxes(bev_tracker_.update(meas, dt), msg->header);
      }
      t_rec = SClock::now();
    }

    if (timing_log_) {
      const auto ms = [](SClock::time_point a, SClock::time_point b) {
          return std::chrono::duration<double, std::milli>(b - a).count();
        };
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "timing ms | conv+cut %.2f (n=%zu) | pubF %.2f | "
        "prep %.2f (n=%zu) | dens %.2f (cells=%zu) | pubD %.2f | "
        "lines %.2f (segs=%zu) | recover %.2f "
        "[acc %.2f seg %.2f fit %.2f n=%d] | total %.2f",
        ms(t0, t_conv), n_elev,
        ms(t_conv, t_pubf), ms(t_pubf, t_prep), cloud->size(),
        ms(t_prep, t_dens), density->size(), ms(t_dens, t_pubd),
        ms(t_pubd, t_lines), n_segs, ms(t_lines, t_rec),
        rec_acc_ms, rec_seg_ms, rec_fit_ms, rec_nseg, ms(t0, t_rec));
    }
  }

  BevMeasurement clusterToBevMeasurement(const Cloud & cloud, const pcl::PointIndices & idx)
  {
    BevMeasurement m;
    double zmn = std::numeric_limits<double>::max();
    double zmx = std::numeric_limits<double>::lowest();
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (int i : idx.indices) {
      const auto & p = cloud.points[i];
      mean += Eigen::Vector2d(p.x, p.y);
      zmn = std::min<double>(zmn, p.z);
      zmx = std::max<double>(zmx, p.z);
    }
    const double n = static_cast<double>(idx.indices.size());
    mean /= n;
    m.z_min = zmn;
    m.z_max = zmx;

    double yaw = 0.0;
    if (idx.indices.size() >= 3) {
      Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
      for (int i : idx.indices) {
        Eigen::Vector2d d(cloud.points[i].x, cloud.points[i].y);
        d -= mean;
        cov += d * d.transpose();
      }
      cov /= (n - 1.0);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
      const Eigen::Vector2d axis = es.eigenvectors().col(1);
      yaw = std::atan2(axis.y(), axis.x());
    }

    const double c = std::cos(yaw), s = std::sin(yaw);
    double umn = std::numeric_limits<double>::max(), vmn = umn;
    double umx = std::numeric_limits<double>::lowest(), vmx = umx;
    for (int i : idx.indices) {
      const double dx = cloud.points[i].x - mean.x();
      const double dy = cloud.points[i].y - mean.y();
      const double u = c * dx + s * dy;
      const double v = -s * dx + c * dy;
      umn = std::min(umn, u); umx = std::max(umx, u);
      vmn = std::min(vmn, v); vmx = std::max(vmx, v);
    }
    m.center = mean + Eigen::Vector2d(c * 0.5 * (umn + umx) - s * 0.5 * (vmn + vmx),
        s * 0.5 * (umn + umx) + c * 0.5 * (vmn + vmx));
    m.half = Eigen::Vector2d(0.5 * (umx - umn), 0.5 * (vmx - vmn));
    m.yaw = yaw;
    return m;
  }

  double computeDt(const std_msgs::msg::Header & header)
  {
    const double now = header.stamp.sec + header.stamp.nanosec * 1e-9;
    double dt = 0.1;
    if (last_stamp_ > 0.0 && now > last_stamp_) {dt = now - last_stamp_;}
    last_stamp_ = now;
    return dt;
  }

  void publishTrackBoxes(
    const std::vector<BevTrack> & tracks, const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(clearMarker(header));
    for (const auto & t : tracks) {
      OBB b;
      b.center = Eigen::Vector3d(t.center.x(), t.center.y(), 0.5 * (t.z_min + t.z_max));
      const double c = std::cos(t.yaw), s = std::sin(t.yaw);
      b.rotation << c, -s, 0,  s, c, 0,  0, 0, 1;
      b.half_extents = Eigen::Vector3d(t.half.x(), t.half.y(), 0.5 * (t.z_max - t.z_min));
      arr.markers.push_back(makeWireBox(b, static_cast<std::size_t>(t.id), header));
    }
    pub_boxes_->publish(arr);
  }

  // Draw the BEV segments as LINE_LISTs at z=0 (sensor height): segments whose
  // length + height band match the target plate size in GREEN ("bev_plates"),
  // all other segments in magenta ("bev_lines"). Both markers are re-published
  // every frame under a FIXED (ns, id), so RViz overwrites them in place — no
  // DELETEALL, whose delete-then-readd gap flickered and whose (ns, id)
  // collided with the magenta marker ("Multiple Markers ... same ns and id").
  // A frame without segments sends DELETE so the stale marker disappears.
  void publishBevLines(
    const std::vector<BevSegment> & segs, const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::MarkerArray arr;
    auto makeLines = [&header](const char * ns, float r, float g, float b, float w) {
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.ns = ns;
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = w;
        m.color.a = 1.0;
        m.color.r = r; m.color.g = g; m.color.b = b;
        m.pose.orientation.w = 1.0;
        return m;
      };
    auto others = makeLines("bev_lines", 1.0f, 0.2f, 1.0f, 0.02f);   // magenta
    auto plates = makeLines("bev_plates", 0.1f, 1.0f, 0.1f, 0.03f);  // green, thicker
    for (const auto & s : segs) {
      geometry_msgs::msg::Point p0, p1;
      p0.x = s.x0; p0.y = s.y0; p0.z = 0.0;
      p1.x = s.x1; p1.y = s.y1; p1.z = 0.0;
      auto & m = isPlateSegment(s, bev_plate_prm_) ? plates : others;
      m.points.push_back(p0);
      m.points.push_back(p1);
    }
    if (others.points.empty()) {others.action = visualization_msgs::msg::Marker::DELETE;}
    if (plates.points.empty()) {plates.action = visualization_msgs::msg::Marker::DELETE;}
    arr.markers.push_back(others);
    arr.markers.push_back(plates);
    pub_bev_lines_->publish(arr);
  }

  visualization_msgs::msg::Marker clearMarker(const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.ns = "boxes";
    clear.id = -1;  // real boxes use ids >= 0 — avoid the (ns, id) collision
                    // RViz rejects ("Multiple Markers ... same ns and id")
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    return clear;
  }

  visualization_msgs::msg::Marker makeWireBox(
    const OBB & b, std::size_t id, const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = "boxes";
    m.id = static_cast<int>(id);
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.01;
    m.color.a = 1.0;
    m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.2;
    m.pose.orientation.w = 1.0;

    auto corner = [&](int i, int j, int k) {
        Eigen::Vector3d local(
          (i ? 1 : -1) * b.half_extents.x(),
          (j ? 1 : -1) * b.half_extents.y(),
          (k ? 1 : -1) * b.half_extents.z());
        Eigen::Vector3d w = b.center + b.rotation * local;
        geometry_msgs::msg::Point p;
        p.x = w.x(); p.y = w.y(); p.z = w.z();
        return p;
      };
    const int edges[12][2][3] = {
      {{0, 0, 0}, {1, 0, 0}}, {{0, 1, 0}, {1, 1, 0}},
      {{0, 0, 1}, {1, 0, 1}}, {{0, 1, 1}, {1, 1, 1}},
      {{0, 0, 0}, {0, 1, 0}}, {{1, 0, 0}, {1, 1, 0}},
      {{0, 0, 1}, {0, 1, 1}}, {{1, 0, 1}, {1, 1, 1}},
      {{0, 0, 0}, {0, 0, 1}}, {{1, 0, 0}, {1, 0, 1}},
      {{0, 1, 0}, {0, 1, 1}}, {{1, 1, 0}, {1, 1, 1}},
    };
    for (const auto & e : edges) {
      m.points.push_back(corner(e[0][0], e[0][1], e[0][2]));
      m.points.push_back(corner(e[1][0], e[1][1], e[1][2]));
    }
    return m;
  }

  BevTrackerParams bev_prm_;
  BevTracker bev_tracker_;
  double last_stamp_ = 0.0;
  std::string input_topic_;
  std::string scalar_field_;
  ElevationFilterParams elev_prm_;
  PreprocessParams pre_;
  bool use_gpu_ = true;
#ifdef OUSTER_CLUSTER_HAS_CUDA
  std::unique_ptr<GpuFrontend> gpu_;
#endif
  BevDensityParams bev_;
  BevLineParams line_prm_;
  BevRefineParams refine_prm_;
  BevPlateParams bev_plate_prm_;
  BevReflGateParams refl_prm_;
  BevSegFilterParams seg_prm_;
  BevSegmentFilter seg_filter_;
  bool recover_enable_ = false;
  PlateRecoverParams recover_prm_;
  int recover_accum_frames_ = 1;      // frames to accumulate for recovery input
  std::deque<Cloud::Ptr> recover_buf_;  // ring of recent density frames
  PlateParams pp_;
  bool timing_log_ = false;  // per-stage latency log (throttled)
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_filtered_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_bev_density_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_bev_lines_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_boxes_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace ouster_cluster

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ouster_cluster::OusterClusterNode>());
  rclcpp::shutdown();
  return 0;
}
