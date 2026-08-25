#include "dynamic_obstacle_tracker/detector/temporal_voxel_map.hpp"

#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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

} // namespace

TemporalVoxelMap::TemporalVoxelMap(const TemporalVoxelMapParams& params) : params_(params)
{
    validateParams(params_);
}

VoxelIndex TemporalVoxelMap::worldToVoxel(const Eigen::Vector3d& point) const
{
    return {static_cast<int>(std::floor(point.x() / params_.voxel_size)),
            static_cast<int>(std::floor(point.y() / params_.voxel_size)),
            static_cast<int>(std::floor(point.z() / params_.voxel_size))};
}

TemporalVoxelMapUpdateResult TemporalVoxelMap::update(
        const VoxelHitMap&     hit_evidence,
        const Eigen::Vector3d& sensor_origin,
        double                 timestamp_sec)
{
    const auto total_start = SteadyClock::now();

    if (!sensor_origin.allFinite())
        throw std::invalid_argument("sensor_origin contains a non-finite value");
    if (!std::isfinite(timestamp_sec) || timestamp_sec <= 0.0)
        throw std::invalid_argument("timestamp_sec must be finite and positive");
    if (last_update_time_ > 0.0 && timestamp_sec <= last_update_time_)
        throw std::invalid_argument("point-cloud timestamps must increase monotonically");

    TemporalVoxelMapUpdateResult result;
    SteadyClock::time_point      ray_casting_end;
    SteadyClock::time_point      classification_end;
    SteadyClock::time_point      free_integration_end;
    SteadyClock::time_point      hit_integration_end;
    SteadyClock::time_point      garbage_collection_end;

    {
        using HitEntry = VoxelHitMap::value_type;
        struct IndexedHitDecision
        {
            VoxelIndex  index;
            HitDecision decision;
        };
        struct FreeBlockUpdate
        {
            VoxelBlock*      block = nullptr;
            const BlockMask* mask  = nullptr;
        };

        std::vector<const HitEntry*> hit_entries;
        hit_entries.reserve(hit_evidence.size());

        BlockMaskMap hit_blocks;
        hit_blocks.reserve(blocks_.size());
        for (const auto& hit : hit_evidence) {
            hit_entries.push_back(&hit);
            const BlockIndex block_index = voxelToBlock(hit.first);
            hit_blocks[block_index].set(voxelOffset(hit.first, block_index));
        }

        BlockMaskMap free_blocks;
        {
            tbb::enumerable_thread_specific<BlockMaskMap> thread_free_blocks;
            tbb::parallel_for(
                    tbb::blocked_range<std::size_t>(0, hit_entries.size(), 64),
                    [&](const tbb::blocked_range<std::size_t>& range) {
                        BlockMaskMap& local_free_blocks = thread_free_blocks.local();
                        for (std::size_t i = range.begin(); i != range.end(); ++i) {
                            const auto& [endpoint, evidence] = *hit_entries[i];
                            traceFreeVoxels(sensor_origin, evidence.representative, endpoint, local_free_blocks);
                        }
                    });

            std::size_t free_block_capacity = 0;
            for (const BlockMaskMap& local_free_blocks : thread_free_blocks)
                free_block_capacity += local_free_blocks.size();
            free_blocks.reserve(free_block_capacity);

            for (const BlockMaskMap& local_free_blocks : thread_free_blocks) {
                for (const auto& [block_index, mask] : local_free_blocks) free_blocks[block_index] |= mask;
            }
        }
        ray_casting_end = SteadyClock::now();

        std::vector<IndexedHitDecision> decisions(hit_entries.size());
        // Preserve scan-batched semantics: every decision reads the map before this scan modifies it.
        tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, hit_entries.size(), 128),
                [&](const tbb::blocked_range<std::size_t>& range) {
                    for (std::size_t i = range.begin(); i != range.end(); ++i) {
                        const auto& [index, evidence]    = *hit_entries[i];
                        auto& [decision_index, decision] = decisions[i];
                        decision_index                   = index;

                        const TemporalVoxel* voxel = findVoxel(index);
                        const bool           continuous_hit
                                = voxel != nullptr && voxel->last_hit_time > 0.0
                               && timestamp_sec - voxel->last_hit_time >= 0.0
                               && timestamp_sec - voxel->last_hit_time <= params_.observation_continuity_timeout;
                        decision.candidate_since           = continuous_hit && voxel->candidate_since > 0.0
                                                                   ? voxel->candidate_since
                                                                   : timestamp_sec;
                        decision.dynamic_observation_count = continuous_hit ? voxel->dynamic_observations + 1 : 1;

                        const bool previously_free = voxel != nullptr && voxel->free_confirmed;
                        const bool recently_dynamic
                                = voxel != nullptr && voxel->last_dynamic_time > 0.0
                               && timestamp_sec - voxel->last_dynamic_time >= 0.0
                               && timestamp_sec - voxel->last_dynamic_time <= params_.dynamic_persistence;
                        const bool enough_points
                                = static_cast<int>(evidence.point_count) >= params_.minimum_points_per_voxel;
                        const bool possible_candidate = enough_points && (previously_free || recently_dynamic);
                        const bool neighborhood_allows_dynamic
                                = !possible_candidate || params_.static_neighbor_threshold == 0
                               || !reachesStaticNeighborThreshold(index, params_.static_neighbor_threshold);

                        decision.candidate = possible_candidate && neighborhood_allows_dynamic;
                        decision.stationary
                                = decision.candidate
                               && timestamp_sec - decision.candidate_since >= params_.occupied_to_static_time;
                        decision.dynamic = decision.candidate && !decision.stationary
                                        && decision.dynamic_observation_count >= params_.minimum_dynamic_observations;
                    }
                });
        classification_end = SteadyClock::now();

        std::vector<FreeBlockUpdate> free_block_updates;
        free_block_updates.reserve(free_blocks.size());
        // Resolve all hash-map insertions first; the parallel loop then gives each task exclusive block ownership.
        for (auto& [block_index, free_mask] : free_blocks) {
            const auto hits = hit_blocks.find(block_index);
            if (hits != hit_blocks.end())
                free_mask &= ~hits->second;
            if (free_mask.none())
                continue;

            auto [block, inserted] = blocks_.try_emplace(block_index);
            static_cast<void>(inserted);
            block->second.last_observed_time = timestamp_sec;
            free_block_updates.push_back({&block->second, &free_mask});
        }

        tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, free_block_updates.size(), 8),
                [&](const tbb::blocked_range<std::size_t>& range) {
                    for (std::size_t i = range.begin(); i != range.end(); ++i) {
                        const FreeBlockUpdate& update = free_block_updates[i];
                        for (std::size_t offset = 0; offset < kVoxelsInBlock; ++offset) {
                            if (update.mask->test(offset))
                                updateFreeVoxel(update.block->voxels[offset], timestamp_sec);
                        }
                    }
                });
        free_integration_end = SteadyClock::now();

        result.dynamic_voxels.reserve(decisions.size());
        for (const auto& [index, decision] : decisions) {
            updateHitVoxel(index, decision, timestamp_sec);
            if (decision.dynamic)
                result.dynamic_voxels.insert(index);
        }
        hit_integration_end = SteadyClock::now();

        ++scan_count_;
        if (scan_count_ % static_cast<std::uint64_t>(params_.garbage_collection_period) == 0)
            result.removed_block_count = garbageCollect(sensor_origin, timestamp_sec);

        result.allocated_block_count = blocks_.size();
        last_update_time_            = timestamp_sec;
        garbage_collection_end       = SteadyClock::now();
    }

    const auto total_end                       = SteadyClock::now();
    result.timings.ray_casting_ms              = elapsedMilliseconds(total_start, ray_casting_end);
    result.timings.candidate_classification_ms = elapsedMilliseconds(ray_casting_end, classification_end);
    result.timings.free_space_integration_ms   = elapsedMilliseconds(classification_end, free_integration_end);
    result.timings.hit_integration_ms          = elapsedMilliseconds(free_integration_end, hit_integration_end);
    result.timings.garbage_collection_ms       = elapsedMilliseconds(hit_integration_end, garbage_collection_end);
    result.timings.scan_cleanup_ms             = elapsedMilliseconds(garbage_collection_end, total_end);
    result.timings.total_ms                    = elapsedMilliseconds(total_start, total_end);
    return result;
}

int TemporalVoxelMap::floorDivide(int value, int divisor)
{
    int quotient  = value / divisor;
    int remainder = value % divisor;
    if (remainder < 0)
        --quotient;
    return quotient;
}

void TemporalVoxelMap::validateParams(const TemporalVoxelMapParams& params)
{
    if (!std::isfinite(params.voxel_size) || params.voxel_size <= 0.0)
        throw std::invalid_argument("voxel_size must be finite and positive");
    if (params.voxels_per_block != kVoxelsPerBlock)
        throw std::invalid_argument("voxels_per_block must be 8");
    if (!(params.minimum_occupancy < params.free_threshold && params.free_threshold < params.occupied_threshold
          && params.occupied_threshold < params.maximum_occupancy)) {
        throw std::invalid_argument(
                "occupancy limits must satisfy minimum < free_threshold < occupied_threshold < maximum");
    }
    if (!(params.hit_increment > 0.0F) || !(params.miss_increment < 0.0F))
        throw std::invalid_argument("hit_increment must be positive and miss_increment must be negative");
    if (params.free_confirmation_time < 0.0 || params.occupied_to_static_time <= 0.0 || params.dynamic_persistence < 0.0
        || params.observation_continuity_timeout <= 0.0) {
        throw std::invalid_argument(
                "temporal durations must be non-negative; occupied-to-static and continuity durations must be "
                "positive");
    }
    if (params.minimum_dynamic_observations < 1 || params.minimum_points_per_voxel < 1)
        throw std::invalid_argument("minimum observation and point counts must be positive");
    if (params.static_neighbor_threshold < 0 || params.static_neighbor_threshold > 26)
        throw std::invalid_argument("static_neighbor_threshold must be in [0, 26]");
    if (!std::isfinite(params.active_radius) || params.active_radius <= 0.0)
        throw std::invalid_argument("active_radius must be finite and positive");
    if (!std::isfinite(params.block_ttl) || params.block_ttl <= 0.0)
        throw std::invalid_argument("block_ttl must be finite and positive");
    if (params.garbage_collection_period < 1)
        throw std::invalid_argument("garbage_collection_period must be positive");
}

TemporalVoxelMap::BlockIndex TemporalVoxelMap::voxelToBlock(const VoxelIndex& index) const
{
    return {floorDivide(index.x, kVoxelsPerBlock),
            floorDivide(index.y, kVoxelsPerBlock),
            floorDivide(index.z, kVoxelsPerBlock)};
}

std::size_t TemporalVoxelMap::voxelOffset(const VoxelIndex& index, const BlockIndex& block) const
{
    const int x = index.x - block.x * kVoxelsPerBlock;
    const int y = index.y - block.y * kVoxelsPerBlock;
    const int z = index.z - block.z * kVoxelsPerBlock;
    return static_cast<std::size_t>(x + kVoxelsPerBlock * (y + kVoxelsPerBlock * z));
}

const TemporalVoxelMap::TemporalVoxel* TemporalVoxelMap::findVoxel(const VoxelIndex& index) const
{
    const BlockIndex block_index = voxelToBlock(index);
    const auto       block       = blocks_.find(block_index);
    if (block == blocks_.end())
        return nullptr;
    return &block->second.voxels[voxelOffset(index, block_index)];
}

TemporalVoxelMap::TemporalVoxel& TemporalVoxelMap::touchVoxel(const VoxelIndex& index, double timestamp_sec)
{
    const BlockIndex block_index = voxelToBlock(index);
    auto [block, inserted]       = blocks_.try_emplace(block_index);
    static_cast<void>(inserted);
    block->second.last_observed_time = timestamp_sec;
    return block->second.voxels[voxelOffset(index, block_index)];
}

void TemporalVoxelMap::traceFreeVoxels(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        const VoxelIndex&      endpoint,
        BlockMaskMap&          free_blocks) const
{
    VoxelIndex current = worldToVoxel(start);
    if (current == endpoint)
        return;

    const Eigen::Vector3d direction = end - start;
    const auto            stepFor   = [](double value) { return value > 0.0 ? 1 : (value < 0.0 ? -1 : 0); };
    const int             step_x    = stepFor(direction.x());
    const int             step_y    = stepFor(direction.y());
    const int             step_z    = stepFor(direction.z());
    const double          infinity  = std::numeric_limits<double>::infinity();

    auto initialMaximum = [&](double origin, double delta, int voxel, int step) {
        if (step == 0)
            return infinity;
        const double boundary = static_cast<double>(voxel + (step > 0 ? 1 : 0)) * params_.voxel_size;
        return (boundary - origin) / delta;
    };
    auto deltaFor = [&](double delta) { return delta == 0.0 ? infinity : params_.voxel_size / std::abs(delta); };

    double       t_max_x       = initialMaximum(start.x(), direction.x(), current.x, step_x);
    double       t_max_y       = initialMaximum(start.y(), direction.y(), current.y, step_y);
    double       t_max_z       = initialMaximum(start.z(), direction.z(), current.z, step_z);
    const double t_delta_x     = deltaFor(direction.x());
    const double t_delta_y     = deltaFor(direction.y());
    const double t_delta_z     = deltaFor(direction.z());
    const int    maximum_steps = static_cast<int>(std::ceil(direction.norm() / params_.voxel_size)) * 3 + 3;

    BlockIndex current_block;
    BlockMask* current_mask     = nullptr;
    bool       has_current_mask = false;

    for (int step_count = 0; step_count < maximum_steps && !(current == endpoint); ++step_count) {
        const BlockIndex block_index = voxelToBlock(current);
        if (!has_current_mask || !(block_index == current_block)) {
            current_block    = block_index;
            current_mask     = &free_blocks[block_index];
            has_current_mask = true;
        }
        current_mask->set(voxelOffset(current, current_block));

        const double next_t = std::min({t_max_x, t_max_y, t_max_z});
        if (!std::isfinite(next_t) || next_t > 1.0)
            break;

        constexpr double kTieTolerance = 1e-12;
        if (std::abs(t_max_x - next_t) <= kTieTolerance) {
            current.x += step_x;
            t_max_x += t_delta_x;
        }
        if (std::abs(t_max_y - next_t) <= kTieTolerance) {
            current.y += step_y;
            t_max_y += t_delta_y;
        }
        if (std::abs(t_max_z - next_t) <= kTieTolerance) {
            current.z += step_z;
            t_max_z += t_delta_z;
        }
    }
}

bool TemporalVoxelMap::reachesStaticNeighborThreshold(const VoxelIndex& index, int threshold) const
{
    int count = 0;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0)
                    continue;
                const TemporalVoxel* neighbor = findVoxel({index.x + dx, index.y + dy, index.z + dz});
                if (neighbor != nullptr && neighbor->static_occupied && ++count >= threshold)
                    return true;
            }
        }
    }
    return false;
}

void TemporalVoxelMap::updateFreeVoxel(TemporalVoxel& voxel, double timestamp_sec)
{
    const bool continuous_free = voxel.last_free_time > 0.0 && timestamp_sec - voxel.last_free_time >= 0.0
                              && timestamp_sec - voxel.last_free_time <= params_.observation_continuity_timeout;

    voxel.last_free_time = timestamp_sec;
    voxel.occupancy      = std::clamp(
            voxel.occupancy + params_.miss_increment, params_.minimum_occupancy, params_.maximum_occupancy);
    voxel.last_hit_time        = 0.0;
    voxel.candidate_since      = 0.0;
    voxel.dynamic_observations = 0;
    voxel.occupied_since       = 0.0;

    if (voxel.occupancy <= params_.free_threshold) {
        if (!continuous_free || voxel.free_since <= 0.0)
            voxel.free_since = timestamp_sec;
        if (timestamp_sec - voxel.free_since >= params_.free_confirmation_time) {
            voxel.free_confirmed  = true;
            voxel.static_occupied = false;
        }
    }
}

void TemporalVoxelMap::updateHitVoxel(const VoxelIndex& index, const HitDecision& decision, double timestamp_sec)
{
    TemporalVoxel& voxel = touchVoxel(index, timestamp_sec);
    voxel.last_hit_time  = timestamp_sec;

    if (decision.candidate && !decision.stationary) {
        voxel.candidate_since      = decision.candidate_since;
        voxel.dynamic_observations = decision.dynamic_observation_count;
        if (decision.dynamic)
            voxel.last_dynamic_time = timestamp_sec;
        return;
    }

    voxel.candidate_since      = 0.0;
    voxel.dynamic_observations = 0;
    voxel.free_since           = 0.0;
    voxel.free_confirmed       = false;
    voxel.occupancy
            = std::clamp(voxel.occupancy + params_.hit_increment, params_.minimum_occupancy, params_.maximum_occupancy);

    if (decision.stationary)
        voxel.occupancy = std::max(voxel.occupancy, params_.occupied_threshold);

    if (voxel.occupancy >= params_.occupied_threshold) {
        if (voxel.occupied_since <= 0.0)
            voxel.occupied_since = timestamp_sec;
        if (decision.stationary || timestamp_sec - voxel.occupied_since >= params_.occupied_to_static_time)
            voxel.static_occupied = true;
    }
}

std::size_t TemporalVoxelMap::garbageCollect(const Eigen::Vector3d& sensor_origin, double timestamp_sec)
{
    std::size_t  removed_count = 0;
    const double block_size    = params_.voxel_size * static_cast<double>(kVoxelsPerBlock);
    const double block_padding = std::sqrt(3.0) * block_size * 0.5;

    for (auto block = blocks_.begin(); block != blocks_.end();) {
        const Eigen::Vector3d block_center(
                (static_cast<double>(block->first.x) + 0.5) * block_size,
                (static_cast<double>(block->first.y) + 0.5) * block_size,
                (static_cast<double>(block->first.z) + 0.5) * block_size);
        const bool stale   = timestamp_sec - block->second.last_observed_time > params_.block_ttl;
        const bool distant = (block_center - sensor_origin).norm() > params_.active_radius + block_padding;
        if (stale || distant) {
            block = blocks_.erase(block);
            ++removed_count;
        } else {
            ++block;
        }
    }
    return removed_count;
}

} // namespace dynamic_obstacle_tracker
