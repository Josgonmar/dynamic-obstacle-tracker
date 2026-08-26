#pragma once

#include <Eigen/Core>
#include <vector>

namespace dynamic_obstacle_tracker {

// Returns one detection index per track, or -1 when leaving that track unmatched is cheaper.
std::vector<int> solveHungarianAssignment(const Eigen::MatrixXd& costs, double unassigned_cost);

} // namespace dynamic_obstacle_tracker
