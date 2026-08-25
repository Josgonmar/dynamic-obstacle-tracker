#include "dynamic_obstacle_tracker/detector/dynamic_point_detector.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dynamic_obstacle_tracker {
namespace {

using SteadyClock = std::chrono::steady_clock;

double elapsedMilliseconds(const SteadyClock::time_point& start, const SteadyClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

struct AcceptedPoint
{
    pcl::PointXYZ point;
    VoxelIndex    voxel;
};

TemporalVoxelMapParams makeTemporalVoxelMapParams(const DynamicPointDetectorParams& params)
{
    TemporalVoxelMapParams map_params;
    map_params.voxel_size                     = params.voxel_size;
    map_params.voxels_per_block               = params.voxels_per_block;
    map_params.hit_increment                  = params.hit_increment;
    map_params.miss_increment                 = params.miss_increment;
    map_params.minimum_occupancy              = params.minimum_occupancy;
    map_params.maximum_occupancy              = params.maximum_occupancy;
    map_params.occupied_threshold             = params.occupied_threshold;
    map_params.free_threshold                 = params.free_threshold;
    map_params.free_confirmation_time         = params.free_confirmation_time;
    map_params.occupied_to_static_time        = params.occupied_to_static_time;
    map_params.dynamic_persistence            = params.dynamic_persistence;
    map_params.observation_continuity_timeout = params.observation_continuity_timeout;
    map_params.minimum_dynamic_observations   = params.minimum_dynamic_observations;
    map_params.minimum_points_per_voxel       = params.minimum_points_per_voxel;
    map_params.static_neighbor_threshold      = params.static_neighbor_threshold;
    map_params.active_radius                  = params.active_radius;
    map_params.block_ttl                      = params.block_ttl;
    map_params.garbage_collection_period      = params.garbage_collection_period;
    return map_params;
}

void validatePreprocessingParams(const DynamicPointDetectorParams& params)
{
    if (!std::isfinite(params.minimum_range) || params.minimum_range < 0.0)
        throw std::invalid_argument("minimum_range must be finite and non-negative");
    if (!std::isfinite(params.maximum_range) || params.maximum_range <= params.minimum_range)
        throw std::invalid_argument("maximum_range must be finite and greater than minimum_range");
}

void finalizeCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    cloud.width    = static_cast<std::uint32_t>(cloud.size());
    cloud.height   = 1;
    cloud.is_dense = true;
}

} // namespace

DynamicPointDetector::DynamicPointDetector(const DynamicPointDetectorParams& params) :
        params_(params),
        voxel_map_(makeTemporalVoxelMapParams(params))
{
    validatePreprocessingParams(params_);
}

DynamicPointDetectionResult DynamicPointDetector::update(
        const pcl::PointCloud<pcl::PointXYZ>& cloud,
        const Eigen::Vector3d&                sensor_origin,
        double                                timestamp_sec,
        bool                                  collect_static_points)
{
    const auto total_start = SteadyClock::now();

    if (!sensor_origin.allFinite())
        throw std::invalid_argument("sensor_origin contains a non-finite value");
    if (!std::isfinite(timestamp_sec) || timestamp_sec <= 0.0)
        throw std::invalid_argument("timestamp_sec must be finite and positive");

    DynamicPointDetectionResult result;
    result.input_point_count = cloud.size();

    std::vector<AcceptedPoint> accepted_points;
    accepted_points.reserve(cloud.size());

    VoxelHitMap hit_evidence;

    const double minimum_range_squared = params_.minimum_range * params_.minimum_range;
    const double maximum_range_squared = params_.maximum_range * params_.maximum_range;

    for (const auto& point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            continue;
        if (params_.remove_ground && point.z <= params_.ground_height + params_.ground_clearance)
            continue;

        const Eigen::Vector3d position(point.x, point.y, point.z);
        const double          range_squared = (position - sensor_origin).squaredNorm();
        if (range_squared < minimum_range_squared || range_squared > maximum_range_squared)
            continue;

        const VoxelIndex endpoint = voxel_map_.worldToVoxel(position);
        accepted_points.push_back({point, endpoint});
        auto [hit, inserted] = hit_evidence.try_emplace(endpoint, VoxelHitEvidence{position, 0});
        static_cast<void>(inserted);
        ++hit->second.point_count;
    }

    result.accepted_point_count  = accepted_points.size();
    const auto preprocessing_end = SteadyClock::now();

    const TemporalVoxelMapUpdateResult map_result     = voxel_map_.update(hit_evidence, sensor_origin, timestamp_sec);
    const auto                         map_update_end = SteadyClock::now();

    result.dynamic_points.reserve(accepted_points.size());
    if (collect_static_points)
        result.static_points.reserve(accepted_points.size());
    for (const auto& accepted : accepted_points) {
        if (map_result.dynamic_voxels.find(accepted.voxel) != map_result.dynamic_voxels.end())
            result.dynamic_points.push_back(accepted.point);
        else if (collect_static_points)
            result.static_points.push_back(accepted.point);
    }
    finalizeCloud(result.dynamic_points);
    if (collect_static_points)
        finalizeCloud(result.static_points);

    result.allocated_block_count = map_result.allocated_block_count;
    result.removed_block_count   = map_result.removed_block_count;
    const auto total_end         = SteadyClock::now();

    result.timings.preprocessing_ms   = elapsedMilliseconds(total_start, preprocessing_end);
    result.timings.map_update_ms      = elapsedMilliseconds(preprocessing_end, map_update_end);
    result.timings.output_assembly_ms = elapsedMilliseconds(map_update_end, total_end);
    result.timings.total_ms           = elapsedMilliseconds(total_start, total_end);
    result.timings.voxel_map          = map_result.timings;
    return result;
}

} // namespace dynamic_obstacle_tracker
