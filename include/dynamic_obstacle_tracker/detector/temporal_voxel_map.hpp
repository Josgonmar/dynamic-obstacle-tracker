#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>

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
    double voxel_size                     = 0.20;
    int    voxels_per_block               = 8;
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

struct TemporalVoxelMapTimings
{
    double ray_casting_ms              = 0.0;
    double candidate_classification_ms = 0.0;
    double free_space_integration_ms   = 0.0;
    double hit_integration_ms          = 0.0;
    double garbage_collection_ms       = 0.0;
    double total_ms                    = 0.0;
};

struct TemporalVoxelMapUpdateResult
{
    VoxelIndexSet           dynamic_voxels;
    std::size_t             allocated_block_count = 0;
    std::size_t             removed_block_count   = 0;
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

    using BlockIndex = VoxelIndex;

    struct TemporalVoxel
    {
        float  occupancy            = 0.0F;
        double last_hit_time        = 0.0;
        double last_free_time       = 0.0;
        double free_since           = 0.0;
        double occupied_since       = 0.0;
        double candidate_since      = 0.0;
        double last_dynamic_time    = 0.0;
        int    dynamic_observations = 0;
        bool   free_confirmed       = false;
        bool   static_occupied      = false;
    };

    struct VoxelBlock
    {
        std::array<TemporalVoxel, kVoxelsInBlock> voxels;
        double                                    last_observed_time = 0.0;
    };

    struct HitDecision
    {
        bool   candidate                 = false;
        bool   dynamic                   = false;
        bool   stationary                = false;
        double candidate_since           = 0.0;
        int    dynamic_observation_count = 0;
    };

    static int  floorDivide(int value, int divisor);
    static void validateParams(const TemporalVoxelMapParams& params);

    BlockIndex           voxelToBlock(const VoxelIndex& index) const;
    std::size_t          voxelOffset(const VoxelIndex& index, const BlockIndex& block) const;
    const TemporalVoxel* findVoxel(const VoxelIndex& index) const;
    TemporalVoxel&       touchVoxel(const VoxelIndex& index, double timestamp_sec);

    void traceFreeVoxels(
            const Eigen::Vector3d& start,
            const Eigen::Vector3d& end,
            const VoxelIndex&      endpoint,
            VoxelIndexSet&         free_voxels) const;
    int         countStaticNeighbors(const VoxelIndex& index) const;
    void        updateFreeVoxel(const VoxelIndex& index, double timestamp_sec);
    void        updateHitVoxel(const VoxelIndex& index, const HitDecision& decision, double timestamp_sec);
    std::size_t garbageCollect(const Eigen::Vector3d& sensor_origin, double timestamp_sec);

    TemporalVoxelMapParams                                     params_;
    std::unordered_map<BlockIndex, VoxelBlock, VoxelIndexHash> blocks_;
    std::uint64_t                                              scan_count_       = 0;
    double                                                     last_update_time_ = 0.0;
};

} // namespace dynamic_obstacle_tracker
