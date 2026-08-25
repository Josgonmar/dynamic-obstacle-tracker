#pragma once

#include <Eigen/Core>
#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace dynamic_obstacle_tracker {

struct VoxelIndex
{
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const VoxelIndex& other) const { return x == other.x && y == other.y && z == other.z; }
};

struct VoxelIndexHash
{
    std::size_t operator()(const VoxelIndex& index) const noexcept
    {
        const auto x = static_cast<std::uint64_t>(static_cast<std::uint32_t>(index.x));
        const auto y = static_cast<std::uint64_t>(static_cast<std::uint32_t>(index.y));
        const auto z = static_cast<std::uint64_t>(static_cast<std::uint32_t>(index.z));
        return static_cast<std::size_t>((x * 73856093ULL) ^ (y * 19349669ULL) ^ (z * 83492791ULL));
    }
};

struct VoxelHitEvidence
{
    Eigen::Vector3d representative = Eigen::Vector3d::Zero();
    std::size_t     point_count    = 0;
};

using VoxelHitMap   = std::unordered_map<VoxelIndex, VoxelHitEvidence, VoxelIndexHash>;
using VoxelIndexSet = std::unordered_set<VoxelIndex, VoxelIndexHash>;

struct TemporalVoxelMapParams
{
    double voxel_size                = 0.20;
    int    voxels_per_block          = 8;
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

struct TemporalVoxelMapTimings
{
    double ray_casting_ms          = 0.0;
    double edf_construction_ms     = 0.0;
    double hmm_update_ms           = 0.0;
    double spatial_convolution_ms  = 0.0;
    double temporal_convolution_ms = 0.0;
    double segmentation_ms         = 0.0;
    double garbage_collection_ms   = 0.0;
    double scan_cleanup_ms         = 0.0;
    double total_ms                = 0.0;
};

struct TemporalVoxelMapUpdateResult
{
    VoxelIndexSet           dynamic_voxels;
    std::size_t             state_change_voxel_count = 0;
    std::size_t             allocated_block_count    = 0;
    std::size_t             removed_block_count      = 0;
    double                  otsu_threshold           = 0.0;
    TemporalVoxelMapTimings timings;
};

class TemporalVoxelMap
{
  public:
    explicit TemporalVoxelMap(const TemporalVoxelMapParams& params);

    VoxelIndex worldToVoxel(const Eigen::Vector3d& point) const;

    TemporalVoxelMapUpdateResult update(
            const VoxelHitMap&     hit_evidence,
            const Eigen::Vector3d& sensor_origin,
            double                 timestamp_sec);

  private:
    static constexpr int kVoxelsPerBlock = 8;
    static constexpr int kVoxelsInBlock  = kVoxelsPerBlock * kVoxelsPerBlock * kVoxelsPerBlock;

    enum class OccupancyState : std::uint8_t {
        Unobserved = 0,
        Occupied   = 1,
        Free       = 2
    };

    using BlockIndex    = VoxelIndex;
    using BlockMask     = std::bitset<kVoxelsInBlock>;
    using BlockMaskMap  = std::unordered_map<BlockIndex, BlockMask, VoxelIndexHash>;
    using VoxelScoreMap = std::unordered_map<VoxelIndex, float, VoxelIndexHash>;

    struct TemporalVoxel
    {
        float          unobserved_probability = 1.0F;
        float          occupied_probability   = 0.0F;
        float          free_probability       = 0.0F;
        std::uint64_t  current_state_scan     = 0;
        std::uint64_t  last_state_change_scan = 0;
        std::uint64_t  last_observed_scan     = 0;
        OccupancyState current_state          = OccupancyState::Unobserved;
    };

    struct VoxelBlock
    {
        std::array<TemporalVoxel, kVoxelsInBlock> voxels;
        std::uint64_t                             last_observed_scan = 0;
    };

    static int  floorDivide(int value, int divisor);
    static void validateParams(const TemporalVoxelMapParams& params);
    static bool computeOtsuThreshold(const std::vector<double>& scores, int histogram_bins, double& threshold);

    BlockIndex           voxelToBlock(const VoxelIndex& index) const;
    std::size_t          voxelOffset(const VoxelIndex& index, const BlockIndex& block) const;
    VoxelIndex           voxelFromOffset(const BlockIndex& block, std::size_t offset) const;
    Eigen::Vector3d      voxelPosition(const VoxelIndex& index) const;
    const TemporalVoxel* findVoxel(const VoxelIndex& index) const;

    void traceObservedVoxels(
            const Eigen::Vector3d& start,
            const Eigen::Vector3d& end,
            const VoxelIndex&      endpoint,
            BlockMaskMap&          observed_blocks) const;
    float       spatialConvolutionScore(const VoxelIndex& index, const VoxelHitMap& hit_evidence) const;
    std::size_t garbageCollect(const Eigen::Vector3d& sensor_origin);

    TemporalVoxelMapParams                                     params_;
    std::unordered_map<BlockIndex, VoxelBlock, VoxelIndexHash> blocks_;
    std::deque<VoxelScoreMap>                                  score_history_;
    VoxelIndexSet                                              previous_hit_voxels_;
    VoxelIndexSet                                              previous_dynamic_voxels_;
    std::uint64_t                                              scan_count_       = 0;
    double                                                     last_update_time_ = 0.0;
};

} // namespace dynamic_obstacle_tracker
