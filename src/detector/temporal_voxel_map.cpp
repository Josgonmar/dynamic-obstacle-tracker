#include "dynamic_obstacle_tracker/detector/temporal_voxel_map.hpp"

#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <chrono>
#include <cilantro/core/kd_tree.hpp>
#include <cmath>
#include <limits>
#include <memory>
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

    ++scan_count_;
    TemporalVoxelMapUpdateResult result;
    SteadyClock::time_point      ray_casting_end;
    SteadyClock::time_point      edf_construction_end;
    SteadyClock::time_point      hmm_update_end;
    SteadyClock::time_point      spatial_convolution_end;
    SteadyClock::time_point      temporal_convolution_end;
    SteadyClock::time_point      segmentation_end;
    SteadyClock::time_point      garbage_collection_end;

    {
        using HitEntry = VoxelHitMap::value_type;
        struct ObservedBlockUpdate
        {
            BlockIndex       index;
            VoxelBlock*      block = nullptr;
            const BlockMask* mask  = nullptr;
        };

        std::vector<const HitEntry*> hit_entries;
        hit_entries.reserve(hit_evidence.size());

        BlockMaskMap observed_blocks;
        observed_blocks.reserve(blocks_.size());
        for (const auto& hit : hit_evidence) {
            hit_entries.push_back(&hit);
            const BlockIndex block_index = voxelToBlock(hit.first);
            observed_blocks[block_index].set(voxelOffset(hit.first, block_index));
        }

        {
            tbb::enumerable_thread_specific<BlockMaskMap> thread_observed_blocks;
            tbb::parallel_for(
                    tbb::blocked_range<std::size_t>(0, hit_entries.size(), 64),
                    [&](const tbb::blocked_range<std::size_t>& range) {
                        BlockMaskMap& local_observed_blocks = thread_observed_blocks.local();
                        for (std::size_t i = range.begin(); i != range.end(); ++i) {
                            const auto& [endpoint, evidence] = *hit_entries[i];
                            traceObservedVoxels(
                                    sensor_origin, evidence.representative, endpoint, local_observed_blocks);
                        }
                    });

            std::size_t observed_block_capacity = observed_blocks.size();
            for (const BlockMaskMap& local_blocks : thread_observed_blocks)
                observed_block_capacity += local_blocks.size();
            observed_blocks.reserve(observed_block_capacity);
            for (const BlockMaskMap& local_blocks : thread_observed_blocks) {
                for (const auto& [block_index, mask] : local_blocks) observed_blocks[block_index] |= mask;
            }
        }
        ray_casting_end = SteadyClock::now();

        VoxelIndexSet edf_voxels;
        edf_voxels.reserve(hit_evidence.size() + previous_hit_voxels_.size());
        for (const auto& [index, evidence] : hit_evidence) {
            static_cast<void>(evidence);
            edf_voxels.insert(index);
        }
        edf_voxels.insert(previous_hit_voxels_.begin(), previous_hit_voxels_.end());

        cilantro::VectorSet3f edf_points(3, static_cast<Eigen::Index>(edf_voxels.size()));
        Eigen::Index          edf_point_index = 0;
        for (const VoxelIndex& index : edf_voxels)
            edf_points.col(edf_point_index++) = voxelPosition(index).cast<float>();

        using EdfTree = cilantro::KDTree3f<cilantro::KDTreeDistanceAdaptors::L2Simple>;
        std::unique_ptr<EdfTree> edf_tree;
        if (edf_points.cols() > 0)
            edf_tree = std::make_unique<EdfTree>(edf_points);
        edf_construction_end = SteadyClock::now();

        std::vector<ObservedBlockUpdate> observed_updates;
        observed_updates.reserve(observed_blocks.size());
        for (const auto& [block_index, mask] : observed_blocks) {
            auto [block, inserted] = blocks_.try_emplace(block_index);
            static_cast<void>(inserted);
            block->second.last_observed_scan = scan_count_;
            observed_updates.push_back({block_index, &block->second, &mask});
        }

        // Stage 1: update the three-state occupancy HMM from the Gaussian EDF likelihood.
        const double inverse_likelihood_denominator = 1.0 / (2.0 * params_.occupancy_sigma * params_.occupancy_sigma);
        tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, observed_updates.size(), 4),
                [&](const tbb::blocked_range<std::size_t>& range) {
                    for (std::size_t i = range.begin(); i != range.end(); ++i) {
                        const ObservedBlockUpdate& update = observed_updates[i];
                        for (std::size_t offset = 0; offset < kVoxelsInBlock; ++offset) {
                            if (!update.mask->test(offset))
                                continue;

                            TemporalVoxel& voxel = update.block->voxels[offset];
                            if (voxel.last_observed_scan > 0
                                && scan_count_ - voxel.last_observed_scan
                                           >= static_cast<std::uint64_t>(params_.global_window_size)) {
                                voxel = TemporalVoxel{};
                            }

                            const VoxelIndex index                = voxelFromOffset(update.index, offset);
                            double           occupancy_likelihood = 0.0;
                            if (edf_voxels.find(index) != edf_voxels.end()) {
                                occupancy_likelihood = 1.0;
                            } else if (edf_tree) {
                                cilantro::Neighbor<float> nearest;
                                const Eigen::Vector3f     query = voxelPosition(index).cast<float>();
                                edf_tree->nearestNeighborSearch(query, nearest);
                                occupancy_likelihood = std::exp(
                                        -static_cast<double>(nearest.value) * inverse_likelihood_denominator);
                            }

                            const double epsilon            = params_.transition_epsilon;
                            const double predicted_occupied = epsilon * voxel.unobserved_probability
                                                            + (1.0 - epsilon) * voxel.occupied_probability
                                                            + epsilon * voxel.free_probability;
                            const double predicted_free = epsilon * voxel.unobserved_probability
                                                        + epsilon * voxel.occupied_probability
                                                        + (1.0 - epsilon) * voxel.free_probability;
                            const double occupied_alpha = occupancy_likelihood * predicted_occupied;
                            const double free_alpha     = (1.0 - occupancy_likelihood) * predicted_free;
                            const double normalization  = occupied_alpha + free_alpha;
                            if (normalization <= std::numeric_limits<double>::epsilon())
                                continue;

                            voxel.unobserved_probability = 0.0F;
                            voxel.occupied_probability   = static_cast<float>(occupied_alpha / normalization);
                            voxel.free_probability       = static_cast<float>(free_alpha / normalization);
                            voxel.last_observed_scan     = scan_count_;
                            voxel.current_state_scan     = scan_count_;

                            const OccupancyState estimated_state = voxel.occupied_probability >= voxel.free_probability
                                                                         ? OccupancyState::Occupied
                                                                         : OccupancyState::Free;
                            const float          state_probability
                                    = std::max(voxel.occupied_probability, voxel.free_probability);
                            if (state_probability <= params_.belief_threshold)
                                continue;
                            if (voxel.current_state != OccupancyState::Unobserved
                                && voxel.current_state != estimated_state) {
                                voxel.last_state_change_scan = scan_count_;
                            }
                            voxel.current_state = estimated_state;
                        }
                    }
                });
        hmm_update_end = SteadyClock::now();

        // Stage 2: turn coherent state changes into dynamic regions using the
        // HMM-MOS spatial and temporal convolutions followed by Otsu segmentation.
        std::vector<float> spatial_scores(hit_entries.size(), 0.0F);
        tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, hit_entries.size(), 128),
                [&](const tbb::blocked_range<std::size_t>& range) {
                    for (std::size_t i = range.begin(); i != range.end(); ++i)
                        spatial_scores[i] = spatialConvolutionScore(hit_entries[i]->first, hit_evidence);
                });

        VoxelScoreMap raw_scores;
        raw_scores.reserve(hit_entries.size());
        for (std::size_t i = 0; i < hit_entries.size(); ++i)
            raw_scores.emplace(hit_entries[i]->first, spatial_scores[i]);

        VoxelScoreMap filtered_scores;
        filtered_scores.reserve(hit_entries.size());
        std::vector<float> filtered_values(hit_entries.size(), 0.0F);
        tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, hit_entries.size(), 128),
                [&](const tbb::blocked_range<std::size_t>& range) {
                    for (std::size_t i = range.begin(); i != range.end(); ++i) {
                        const VoxelIndex&  index = hit_entries[i]->first;
                        std::vector<float> neighborhood_scores;
                        neighborhood_scores.reserve(27);
                        for (int dx = -1; dx <= 1; ++dx) {
                            for (int dy = -1; dy <= 1; ++dy) {
                                for (int dz = -1; dz <= 1; ++dz) {
                                    const auto score = raw_scores.find({index.x + dx, index.y + dy, index.z + dz});
                                    if (score != raw_scores.end())
                                        neighborhood_scores.push_back(score->second);
                                }
                            }
                        }
                        const std::size_t middle = neighborhood_scores.size() / 2;
                        std::nth_element(
                                neighborhood_scores.begin(),
                                neighborhood_scores.begin() + static_cast<std::ptrdiff_t>(middle),
                                neighborhood_scores.end());
                        float median = neighborhood_scores[middle];
                        if (neighborhood_scores.size() % 2 == 0) {
                            const float lower = *std::max_element(
                                    neighborhood_scores.begin(),
                                    neighborhood_scores.begin() + static_cast<std::ptrdiff_t>(middle));
                            median = 0.5F * (lower + median);
                        }
                        filtered_values[i] = median;
                    }
                });
        for (std::size_t i = 0; i < hit_entries.size(); ++i)
            filtered_scores.emplace(hit_entries[i]->first, filtered_values[i]);
        spatial_convolution_end = SteadyClock::now();

        score_history_.push_back(std::move(filtered_scores));
        while (score_history_.size() > static_cast<std::size_t>(params_.local_window_size)) score_history_.pop_front();

        std::vector<double> temporal_scores(hit_entries.size(), 0.0);
        tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, hit_entries.size(), 128),
                [&](const tbb::blocked_range<std::size_t>& range) {
                    for (std::size_t i = range.begin(); i != range.end(); ++i) {
                        const VoxelIndex& index = hit_entries[i]->first;
                        double            score = 0.0;
                        for (const VoxelScoreMap& history : score_history_) {
                            const auto historic_score = history.find(index);
                            if (historic_score != history.end())
                                score += historic_score->second;
                        }
                        temporal_scores[i] = score;
                    }
                });
        temporal_convolution_end = SteadyClock::now();

        VoxelScoreMap current_temporal_scores;
        current_temporal_scores.reserve(hit_entries.size());
        for (std::size_t i = 0; i < hit_entries.size(); ++i)
            current_temporal_scores.emplace(hit_entries[i]->first, static_cast<float>(temporal_scores[i]));

        VoxelIndexSet high_confidence_dynamic;
        const bool    threshold_valid
                = scan_count_ > static_cast<std::uint64_t>(params_.local_window_size)
               && score_history_.size() == static_cast<std::size_t>(params_.local_window_size)
               && computeOtsuThreshold(temporal_scores, params_.histogram_bins, result.otsu_threshold);
        if (threshold_valid) {
            high_confidence_dynamic.reserve(hit_entries.size());
            for (std::size_t i = 0; i < hit_entries.size(); ++i) {
                if (temporal_scores[i] > result.otsu_threshold)
                    high_confidence_dynamic.insert(hit_entries[i]->first);
            }
        }

        for (const VoxelIndex& index : previous_dynamic_voxels_) {
            const auto score = current_temporal_scores.find(index);
            if (score != current_temporal_scores.end() && score->second > params_.minimum_otsu_threshold)
                high_confidence_dynamic.insert(index);
        }

        VoxelIndexSet dilated_dynamic = high_confidence_dynamic;
        for (const VoxelIndex& index : high_confidence_dynamic) {
            for (int dx = -params_.dilation_radius_voxels; dx <= params_.dilation_radius_voxels; ++dx) {
                for (int dy = -params_.dilation_radius_voxels; dy <= params_.dilation_radius_voxels; ++dy) {
                    for (int dz = -params_.dilation_radius_voxels; dz <= params_.dilation_radius_voxels; ++dz) {
                        const VoxelIndex neighbor{index.x + dx, index.y + dy, index.z + dz};
                        if (hit_evidence.find(neighbor) != hit_evidence.end())
                            dilated_dynamic.insert(neighbor);
                    }
                }
            }
        }
        previous_dynamic_voxels_ = dilated_dynamic;
        if (threshold_valid && result.otsu_threshold > params_.minimum_otsu_threshold)
            result.dynamic_voxels = std::move(dilated_dynamic);

        for (const auto& [index, evidence] : hit_evidence) {
            static_cast<void>(evidence);
            const TemporalVoxel* voxel = findVoxel(index);
            if (voxel != nullptr && voxel->last_state_change_scan == scan_count_)
                ++result.state_change_voxel_count;
        }

        previous_hit_voxels_.clear();
        previous_hit_voxels_.reserve(hit_evidence.size());
        for (const auto& [index, evidence] : hit_evidence) {
            static_cast<void>(evidence);
            previous_hit_voxels_.insert(index);
        }
        segmentation_end = SteadyClock::now();

        if (scan_count_ % static_cast<std::uint64_t>(params_.garbage_collection_period) == 0)
            result.removed_block_count = garbageCollect(sensor_origin);
        result.allocated_block_count = blocks_.size();
        last_update_time_            = timestamp_sec;
        garbage_collection_end       = SteadyClock::now();
    }

    const auto total_end                   = SteadyClock::now();
    result.timings.ray_casting_ms          = elapsedMilliseconds(total_start, ray_casting_end);
    result.timings.edf_construction_ms     = elapsedMilliseconds(ray_casting_end, edf_construction_end);
    result.timings.hmm_update_ms           = elapsedMilliseconds(edf_construction_end, hmm_update_end);
    result.timings.spatial_convolution_ms  = elapsedMilliseconds(hmm_update_end, spatial_convolution_end);
    result.timings.temporal_convolution_ms = elapsedMilliseconds(spatial_convolution_end, temporal_convolution_end);
    result.timings.segmentation_ms         = elapsedMilliseconds(temporal_convolution_end, segmentation_end);
    result.timings.garbage_collection_ms   = elapsedMilliseconds(segmentation_end, garbage_collection_end);
    result.timings.scan_cleanup_ms         = elapsedMilliseconds(garbage_collection_end, total_end);
    result.timings.total_ms                = elapsedMilliseconds(total_start, total_end);
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
    if (!std::isfinite(params.occupancy_sigma) || params.occupancy_sigma <= 0.0)
        throw std::invalid_argument("occupancy_sigma must be finite and positive");
    if (!std::isfinite(params.belief_threshold) || params.belief_threshold <= 0.5 || params.belief_threshold >= 1.0) {
        throw std::invalid_argument("belief_threshold must be in (0.5, 1.0)");
    }
    if (!std::isfinite(params.transition_epsilon) || params.transition_epsilon <= 0.0
        || params.transition_epsilon >= 0.5) {
        throw std::invalid_argument("transition_epsilon must be in (0.0, 0.5)");
    }
    if (params.convolution_size < 1 || params.convolution_size % 2 == 0)
        throw std::invalid_argument("convolution_size must be a positive odd integer");
    if (params.local_window_size < 1 || params.global_window_size < params.local_window_size)
        throw std::invalid_argument("global_window_size must be at least local_window_size, both positive");
    if (!std::isfinite(params.minimum_otsu_threshold) || params.minimum_otsu_threshold < 0.0)
        throw std::invalid_argument("minimum_otsu_threshold must be finite and non-negative");
    if (params.dilation_radius_voxels < 0)
        throw std::invalid_argument("dilation_radius_voxels must be non-negative");
    if (params.histogram_bins < 2)
        throw std::invalid_argument("histogram_bins must be at least 2");
    if (!std::isfinite(params.active_radius) || params.active_radius <= 0.0)
        throw std::invalid_argument("active_radius must be finite and positive");
    if (params.garbage_collection_period < 1)
        throw std::invalid_argument("garbage_collection_period must be positive");
}

bool TemporalVoxelMap::computeOtsuThreshold(const std::vector<double>& scores, int histogram_bins, double& threshold)
{
    if (scores.empty())
        return false;
    const auto [minimum, maximum] = std::minmax_element(scores.begin(), scores.end());
    if (*maximum - *minimum <= std::numeric_limits<double>::epsilon())
        return false;

    const double        bin_size = (*maximum - *minimum) / static_cast<double>(histogram_bins);
    std::vector<double> histogram(static_cast<std::size_t>(histogram_bins), 0.0);
    for (double score : scores) {
        int bin = 0;
        if (score - *minimum >= 1e-3 && *maximum - score < 1e-3) {
            bin = histogram_bins - 1;
        } else if (score - *minimum >= 1e-3) {
            bin = static_cast<int>(std::ceil((score - *minimum) / bin_size)) - 1;
        }
        bin = std::clamp(bin, 0, histogram_bins - 1);
        histogram[static_cast<std::size_t>(bin)] += 1.0;
    }

    const double total        = static_cast<double>(scores.size());
    double       weighted_sum = 0.0;
    for (int i = 0; i < histogram_bins; ++i)
        weighted_sum += static_cast<double>(i) * histogram[static_cast<std::size_t>(i)];

    double background_weight = 0.0;
    double background_sum    = 0.0;
    double maximum_variance  = -1.0;
    int    selected_level    = -1;
    for (int i = 0; i < histogram_bins; ++i) {
        const double foreground_weight = total - background_weight;
        if (background_weight > 0.0 && foreground_weight > 0.0) {
            const double background_mean = background_sum / background_weight;
            const double foreground_mean = (weighted_sum - background_sum) / foreground_weight;
            const double difference      = background_mean - foreground_mean;
            const double variance        = background_weight * foreground_weight * difference * difference;
            if (variance >= maximum_variance) {
                maximum_variance = variance;
                selected_level   = i + 1;
            }
        }
        background_weight += histogram[static_cast<std::size_t>(i)];
        background_sum += static_cast<double>(i) * histogram[static_cast<std::size_t>(i)];
    }
    if (selected_level <= 0)
        return false;
    threshold = *minimum + static_cast<double>(selected_level - 1) * bin_size;
    return true;
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

VoxelIndex TemporalVoxelMap::voxelFromOffset(const BlockIndex& block, std::size_t offset) const
{
    const int x = static_cast<int>(offset % kVoxelsPerBlock);
    offset /= kVoxelsPerBlock;
    const int y = static_cast<int>(offset % kVoxelsPerBlock);
    const int z = static_cast<int>(offset / kVoxelsPerBlock);
    return {block.x * kVoxelsPerBlock + x, block.y * kVoxelsPerBlock + y, block.z * kVoxelsPerBlock + z};
}

Eigen::Vector3d TemporalVoxelMap::voxelPosition(const VoxelIndex& index) const
{
    return params_.voxel_size * Eigen::Vector3d(index.x, index.y, index.z);
}

const TemporalVoxelMap::TemporalVoxel* TemporalVoxelMap::findVoxel(const VoxelIndex& index) const
{
    const BlockIndex block_index = voxelToBlock(index);
    const auto       block       = blocks_.find(block_index);
    if (block == blocks_.end())
        return nullptr;
    return &block->second.voxels[voxelOffset(index, block_index)];
}

void TemporalVoxelMap::traceObservedVoxels(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        const VoxelIndex&      endpoint,
        BlockMaskMap&          observed_blocks) const
{
    VoxelIndex            current   = worldToVoxel(start);
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
    for (int step_count = 0; step_count < maximum_steps; ++step_count) {
        const BlockIndex block_index = voxelToBlock(current);
        if (!has_current_mask || !(block_index == current_block)) {
            current_block    = block_index;
            current_mask     = &observed_blocks[block_index];
            has_current_mask = true;
        }
        current_mask->set(voxelOffset(current, current_block));
        if (current == endpoint)
            break;

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

float TemporalVoxelMap::spatialConvolutionScore(const VoxelIndex& index, const VoxelHitMap& hit_evidence) const
{
    const int edge  = (params_.convolution_size - 1) / 2;
    float     score = 0.0F;
    for (int dx = -edge; dx <= edge; ++dx) {
        for (int dy = -edge; dy <= edge; ++dy) {
            for (int dz = -edge; dz <= edge; ++dz) {
                const VoxelIndex     neighbor{index.x + dx, index.y + dy, index.z + dz};
                const TemporalVoxel* voxel  = findVoxel(neighbor);
                const bool           recent = voxel != nullptr && voxel->current_state != OccupancyState::Unobserved
                                 && scan_count_ - voxel->current_state_scan
                                            < static_cast<std::uint64_t>(params_.global_window_size);
                if (!recent) {
                    score -= 1.0F;
                    continue;
                }
                if (voxel->last_state_change_scan == scan_count_ && hit_evidence.find(neighbor) != hit_evidence.end()) {
                    score += 1.0F;
                }
            }
        }
    }
    return std::max(score, 0.0F);
}

std::size_t TemporalVoxelMap::garbageCollect(const Eigen::Vector3d& sensor_origin)
{
    std::size_t  removed_count = 0;
    const double block_size    = params_.voxel_size * static_cast<double>(kVoxelsPerBlock);
    const double block_padding = std::sqrt(3.0) * block_size * 0.5;
    for (auto block = blocks_.begin(); block != blocks_.end();) {
        const Eigen::Vector3d block_center(
                (static_cast<double>(block->first.x) + 0.5) * block_size,
                (static_cast<double>(block->first.y) + 0.5) * block_size,
                (static_cast<double>(block->first.z) + 0.5) * block_size);
        const bool stale = scan_count_ - block->second.last_observed_scan
                        >= static_cast<std::uint64_t>(params_.global_window_size);
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
