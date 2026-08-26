#include "dynamic_obstacle_tracker/tracker/data_association.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dynamic_obstacle_tracker {

std::vector<int> solveHungarianAssignment(const Eigen::MatrixXd& costs, double unassigned_cost)
{
    if (!std::isfinite(unassigned_cost) || unassigned_cost <= 0.0)
        throw std::invalid_argument("unassigned_cost must be finite and positive");

    const int        track_count     = static_cast<int>(costs.rows());
    const int        detection_count = static_cast<int>(costs.cols());
    std::vector<int> assignment(static_cast<std::size_t>(track_count), -1);
    if (track_count == 0)
        return assignment;

    // One private-equivalent dummy column per track guarantees that every row can remain unmatched.
    const int    column_count = detection_count + track_count;
    const double forbidden_cost
            = std::max(unassigned_cost * static_cast<double>(track_count + 1), unassigned_cost + 1.0);

    auto cost = [&](int track, int column) {
        if (column >= detection_count)
            return unassigned_cost;
        const double value = costs(track, column);
        return std::isfinite(value) && value < unassigned_cost ? value : forbidden_cost;
    };

    // Rectangular Hungarian algorithm. There are always at least as many columns as rows.
    std::vector<double> potential_rows(static_cast<std::size_t>(track_count + 1), 0.0);
    std::vector<double> potential_columns(static_cast<std::size_t>(column_count + 1), 0.0);
    std::vector<int>    matched_row(static_cast<std::size_t>(column_count + 1), 0);
    std::vector<int>    previous_column(static_cast<std::size_t>(column_count + 1), 0);

    for (int row = 1; row <= track_count; ++row) {
        matched_row[0]             = row;
        int                 column = 0;
        std::vector<double> minimum_reduced_cost(
                static_cast<std::size_t>(column_count + 1), std::numeric_limits<double>::infinity());
        std::vector<bool> visited(static_cast<std::size_t>(column_count + 1), false);

        do {
            visited[static_cast<std::size_t>(column)] = true;
            const int current_row                     = matched_row[static_cast<std::size_t>(column)];
            double    delta                           = std::numeric_limits<double>::infinity();
            int       next_column                     = 0;

            for (int candidate = 1; candidate <= column_count; ++candidate) {
                if (visited[static_cast<std::size_t>(candidate)])
                    continue;

                const double reduced_cost = cost(current_row - 1, candidate - 1)
                                          - potential_rows[static_cast<std::size_t>(current_row)]
                                          - potential_columns[static_cast<std::size_t>(candidate)];
                if (reduced_cost < minimum_reduced_cost[static_cast<std::size_t>(candidate)]) {
                    minimum_reduced_cost[static_cast<std::size_t>(candidate)] = reduced_cost;
                    previous_column[static_cast<std::size_t>(candidate)]      = column;
                }
                if (minimum_reduced_cost[static_cast<std::size_t>(candidate)] < delta) {
                    delta       = minimum_reduced_cost[static_cast<std::size_t>(candidate)];
                    next_column = candidate;
                }
            }

            for (int candidate = 0; candidate <= column_count; ++candidate) {
                if (visited[static_cast<std::size_t>(candidate)]) {
                    potential_rows[static_cast<std::size_t>(matched_row[static_cast<std::size_t>(candidate)])] += delta;
                    potential_columns[static_cast<std::size_t>(candidate)] -= delta;
                } else {
                    minimum_reduced_cost[static_cast<std::size_t>(candidate)] -= delta;
                }
            }
            column = next_column;
        } while (matched_row[static_cast<std::size_t>(column)] != 0);

        do {
            const int preceding                           = previous_column[static_cast<std::size_t>(column)];
            matched_row[static_cast<std::size_t>(column)] = matched_row[static_cast<std::size_t>(preceding)];
            column                                        = preceding;
        } while (column != 0);
    }

    for (int column = 1; column <= detection_count; ++column) {
        const int row = matched_row[static_cast<std::size_t>(column)];
        if (row > 0 && costs(row - 1, column - 1) < unassigned_cost)
            assignment[static_cast<std::size_t>(row - 1)] = column - 1;
    }
    return assignment;
}

} // namespace dynamic_obstacle_tracker
