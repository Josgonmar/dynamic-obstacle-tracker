#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <cstddef>
#include <memory>

namespace dynamic_obstacle_tracker {

struct DynamicPointDetectorParams
{
    double voxel_size                     = 0.20;
    int    voxels_per_block               = 8;
    double minimum_range                  = 0.50;
    double maximum_range                  = 30.0;
    bool   remove_ground                  = false;
    double ground_height                  = 0.0;
    double ground_clearance               = 0.15;
    float  hit_increment                  = 0.40F;
    float  miss_increment                 = -0.40F;
    float  minimum_occupancy              = -1.0F;
    float  maximum_occupancy              = 1.0F;
    float  occupied_threshold             = 0.60F;
    float  free_threshold                 = -0.20F;
    double free_confirmation_time         = 0.20;
    double occupied_to_static_time        = 2.0;
    double dynamic_persistence            = 0.50;
    double observation_continuity_timeout = 0.50;
    int    minimum_dynamic_observations   = 2;
    int    minimum_points_per_voxel       = 1;
    int    static_neighbor_threshold      = 5;
    double active_radius                  = 40.0;
    double block_ttl                      = 10.0;
    int    garbage_collection_period      = 10;
};

struct DynamicPointDetectionResult
{
    pcl::PointCloud<pcl::PointXYZ> dynamic_points;
    pcl::PointCloud<pcl::PointXYZ> static_points;
    std::size_t                    input_point_count     = 0;
    std::size_t                    accepted_point_count  = 0;
    std::size_t                    allocated_block_count = 0;
    std::size_t                    removed_block_count   = 0;
};

class DynamicPointDetector
{
  public:
    explicit DynamicPointDetector(const DynamicPointDetectorParams& params);
    ~DynamicPointDetector();

    DynamicPointDetector(const DynamicPointDetector&)            = delete;
    DynamicPointDetector& operator=(const DynamicPointDetector&) = delete;
    DynamicPointDetector(DynamicPointDetector&&)                 = delete;
    DynamicPointDetector& operator=(DynamicPointDetector&&)      = delete;

    DynamicPointDetectionResult update(
            const pcl::PointCloud<pcl::PointXYZ>& cloud,
            const Eigen::Vector3d&                sensor_origin,
            double                                timestamp_sec);

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace dynamic_obstacle_tracker
