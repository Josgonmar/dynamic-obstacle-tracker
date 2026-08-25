#pragma once

#include <Eigen/Core>
#include <cilantro/utilities/point_cloud.hpp>
#include <cstddef>

#include "dynamic_obstacle_tracker/detector/temporal_voxel_map.hpp"

namespace dynamic_obstacle_tracker {

struct DynamicPointDetectorParams
{
    double voxel_size                = 0.20;
    int    voxels_per_block          = 8;
    double minimum_range             = 0.50;
    double maximum_range             = 30.0;
    bool   remove_ground             = false;
    double ground_height             = 0.0;
    double ground_clearance          = 0.15;
    double occupancy_sigma           = 0.20;
    double belief_threshold          = 0.99;
    double transition_epsilon        = 0.005;
    int    convolution_size          = 5;
    int    local_window_size         = 3;
    int    global_window_size        = 300;
    double minimum_otsu_threshold    = 3.0;
    int    dilation_radius_voxels    = 1;
    int    histogram_bins            = 100;
    double active_radius             = 40.0;
    int    garbage_collection_period = 10;
};

struct DynamicPointDetectorTimings
{
    double                  preprocessing_ms   = 0.0;
    double                  map_update_ms      = 0.0;
    double                  output_assembly_ms = 0.0;
    double                  total_ms           = 0.0;
    TemporalVoxelMapTimings voxel_map;
};

struct DynamicPointDetectionResult
{
    cilantro::PointCloud3f      dynamic_points;
    cilantro::PointCloud3f      static_points;
    std::size_t                 input_point_count        = 0;
    std::size_t                 accepted_point_count     = 0;
    std::size_t                 state_change_voxel_count = 0;
    std::size_t                 dynamic_voxel_count      = 0;
    std::size_t                 allocated_block_count    = 0;
    std::size_t                 removed_block_count      = 0;
    double                      otsu_threshold           = 0.0;
    DynamicPointDetectorTimings timings;
};

class DynamicPointDetector
{
  public:
    explicit DynamicPointDetector(const DynamicPointDetectorParams& params);

    DynamicPointDetector(const DynamicPointDetector&)            = delete;
    DynamicPointDetector& operator=(const DynamicPointDetector&) = delete;
    DynamicPointDetector(DynamicPointDetector&&)                 = delete;
    DynamicPointDetector& operator=(DynamicPointDetector&&)      = delete;

    DynamicPointDetectionResult update(
            const cilantro::PointCloud3f& cloud,
            const Eigen::Vector3d&        sensor_origin,
            double                        timestamp_sec,
            bool                          collect_static_points = true);

  private:
    DynamicPointDetectorParams params_;
    TemporalVoxelMap           voxel_map_;
};

} // namespace dynamic_obstacle_tracker
