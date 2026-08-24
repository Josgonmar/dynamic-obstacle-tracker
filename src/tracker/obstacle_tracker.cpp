#include "dynamic_obstacle_tracker/tracker/obstacle_tracker.hpp"

#include <pcl/search/kdtree.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/msg/point.hpp>
#include <limits>

namespace dynamic_obstacle_tracker {

msg::PiecewisePolynomial3 convertPiecewisePolynomialMessage(const PiecewisePolynomial& pwp)
{
    msg::PiecewisePolynomial3 pwp_msg;
    pwp_msg.times = pwp.times;

    for (const auto& coefficients : pwp.coeff_x) {
        msg::Polynomial3 coefficient;
        coefficient.a = coefficients(0);
        coefficient.b = coefficients(1);
        coefficient.c = coefficients(2);
        coefficient.d = coefficients(3);
        pwp_msg.coeff_x.push_back(coefficient);
    }
    for (const auto& coefficients : pwp.coeff_y) {
        msg::Polynomial3 coefficient;
        coefficient.a = coefficients(0);
        coefficient.b = coefficients(1);
        coefficient.c = coefficients(2);
        coefficient.d = coefficients(3);
        pwp_msg.coeff_y.push_back(coefficient);
    }
    for (const auto& coefficients : pwp.coeff_z) {
        msg::Polynomial3 coefficient;
        coefficient.a = coefficients(0);
        coefficient.b = coefficients(1);
        coefficient.c = coefficients(2);
        coefficient.d = coefficients(3);
        pwp_msg.coeff_z.push_back(coefficient);
    }
    return pwp_msg;
}

EKFState::EKFState(
        int                    state_size,
        const Eigen::MatrixXd& Q_init,
        const Eigen::MatrixXd& R_init,
        double                 time,
        const Eigen::Vector3d& bbox_init,
        int                    id_val)
{
    x            = Eigen::VectorXd::Zero(state_size);
    P            = Eigen::MatrixXd::Identity(state_size, state_size);
    Q            = Q_init;
    R            = R_init;
    time_updated = time;
    bbox         = bbox_init;
    id           = id_val;

    color.r = static_cast<float>(rand()) / RAND_MAX;
    color.g = static_cast<float>(rand()) / RAND_MAX;
    color.b = static_cast<float>(rand()) / RAND_MAX;
    color.a = 0.4F;
}

ObstacleTracker::ObstacleTracker(const ObstacleTrackerParams& params, rclcpp::Logger logger) :
        params_(params),
        logger_(logger)
{}

void ObstacleTracker::setFrameId(std::string frame_id)
{
    params_.frame_id = std::move(frame_id);
}

TrackingResult ObstacleTracker::update(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& dynamic_cloud,
        double                                     current_time_sec)
{
    TrackingResult result;

    deleteOldStates(current_time_sec);

    if (!dynamic_cloud || dynamic_cloud->empty())
        return result;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(dynamic_cloud);

    std::vector<pcl::PointIndices>                 cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
    cluster_extraction.setClusterTolerance(params_.cluster_tolerance);
    cluster_extraction.setMinClusterSize(params_.min_cluster_size);
    cluster_extraction.setMaxClusterSize(params_.max_cluster_size);
    cluster_extraction.setSearchMethod(tree);
    cluster_extraction.setInputCloud(dynamic_cloud);
    cluster_extraction.extract(cluster_indices);

    std::vector<Eigen::Vector3d> centroids;
    std::vector<Eigen::Vector3d> bboxes;
    getCentroidsAndSizes(dynamic_cloud, cluster_indices, centroids, bboxes);
    std::vector<Cluster> clusters;
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        const Eigen::Vector3d& centroid = centroids[i];
        const Eigen::Vector3d& bbox     = bboxes[i];

        if (bbox.maxCoeff() > params_.cluster_bbox_cutoff_size)
            continue;

        const int closest_idx = associateCluster(centroid, ekf_states_, params_.cluster_tolerance);

        Cluster cluster;
        if (closest_idx >= 0) {
            double actual_dt = current_time_sec - ekf_states_[closest_idx].time_updated;
            if (actual_dt < 1e-6)
                actual_dt = params_.adaptive_kf_dt;

            ekfPredict(ekf_states_[closest_idx], actual_dt);
            aekfUpdate(
                    ekf_states_[closest_idx],
                    centroid,
                    params_.adaptive_kf_alpha,
                    current_time_sec,
                    bbox,
                    params_.use_adaptive_kf);
            appendToHistory(ekf_states_[closest_idx], current_time_sec, centroid);

            cluster.ekf_state = ekf_states_[closest_idx];
            cluster.centroid  = centroid;
        } else {
            Eigen::MatrixXd Q_avg;
            Eigen::MatrixXd R_avg;
            calculateAverageQandR(Q_avg, R_avg);
            EKFState new_state(9, Q_avg, R_avg, current_time_sec, bbox, next_ekf_id_++);
            new_state.x.head(3) = centroid;
            appendToHistory(new_state, current_time_sec, centroid);
            ekf_states_.push_back(new_state);

            cluster.ekf_state = new_state;
            cluster.centroid  = centroid;
        }

        clusters.push_back(cluster);
    }

    generateBoxMarkers(clusters, result);
    generatePredictions(clusters, current_time_sec, result);
    return result;
}

void ObstacleTracker::appendToHistory(EKFState& state, double time, const Eigen::Vector3d& pos)
{
    state.position_history.emplace_back(time, pos);
    if (static_cast<int>(state.position_history.size()) > params_.max_history_size) {
        state.position_history.erase(state.position_history.begin());
    }
}

void ObstacleTracker::ekfPredict(EKFState& state, double dt)
{
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
    F(0, 3)           = dt;
    F(1, 4)           = dt;
    F(2, 5)           = dt;
    F(0, 6)           = 0.5 * dt * dt;
    F(1, 7)           = 0.5 * dt * dt;
    F(2, 8)           = 0.5 * dt * dt;
    F(3, 6)           = dt;
    F(4, 7)           = dt;
    F(5, 8)           = dt;

    state.x = F * state.x;
    state.P = F * state.P.selfadjointView<Eigen::Lower>() * F.transpose() + state.Q;
}

void ObstacleTracker::aekfUpdate(
        EKFState&              state,
        const Eigen::VectorXd& z,
        double                 alpha,
        double                 time_updated,
        const Eigen::Vector3d& bbox,
        bool                   use_adaptive)
{
    Eigen::MatrixXd H(3, 9);
    H << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

    const Eigen::VectorXd innovation            = z - H * state.x;
    const Eigen::MatrixXd innovation_covariance = H * state.P * H.transpose() + state.R;
    const Eigen::MatrixXd gain                  = state.P * H.transpose() * innovation_covariance.inverse();

    state.x                        = state.x + gain * innovation;
    const Eigen::VectorXd residual = z - H * state.x;

    if (use_adaptive) {
        state.R = alpha * state.R + (1.0 - alpha) * (residual * residual.transpose() + H * state.P * H.transpose());
        state.Q = alpha * state.Q + (1.0 - alpha) * (gain * innovation * innovation.transpose() * gain.transpose());
    } else {
        state.R = Eigen::MatrixXd::Identity(3, 3) * 0.1;
        state.Q = Eigen::MatrixXd::Identity(9, 9) * 0.01;
    }

    state.P            = (Eigen::MatrixXd::Identity(9, 9) - gain * H) * state.P;
    state.time_updated = time_updated;
    state.bbox         = 0.5 * state.bbox + 0.5 * bbox;
}

int ObstacleTracker::associateCluster(
        const Eigen::Vector3d&       centroid,
        const std::vector<EKFState>& states,
        double                       tolerance)
{
    double min_distance = tolerance;
    int    closest_idx  = -1;

    for (size_t i = 0; i < states.size(); ++i) {
        const Eigen::Vector3d state_position(states[i].x[0], states[i].x[1], states[i].x[2]);
        const double          distance = (centroid - state_position).norm();
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx  = static_cast<int>(i);
        }
    }
    return closest_idx;
}

Eigen::VectorXd ObstacleTracker::polyfit(const std::vector<double>& t, const std::vector<double>& y, int degree)
{
    const int n = static_cast<int>(t.size());
    if (degree >= n)
        degree = n - 1;

    Eigen::MatrixXd X(n, degree + 1);
    Eigen::VectorXd Y(n);
    for (int i = 0; i < n; ++i) {
        Y(i) = y[i];
        for (int j = 0; j <= degree; ++j) X(i, j) = std::pow(t[i], degree - j);
    }
    return (X.transpose() * X).ldlt().solve(X.transpose() * Y);
}

double ObstacleTracker::calculateVariance(
        const std::vector<double>& t,
        const std::vector<double>& y,
        const Eigen::VectorXd&     beta,
        int                        degree)
{
    const int n            = static_cast<int>(t.size());
    double    residual_sum = 0.0;
    for (int i = 0; i < n; ++i) {
        double fitted = 0.0;
        for (int j = 0; j <= degree; ++j) fitted += beta(j) * std::pow(t[i], degree - j);
        const double residual = y[i] - fitted;
        residual_sum += residual * residual;
    }
    const int denominator = n - degree - 1;
    return (denominator > 0) ? residual_sum / denominator : 0.0;
}

void ObstacleTracker::deleteOldStates(double current_time)
{
    for (auto it = ekf_states_.begin(); it != ekf_states_.end();) {
        if (current_time - it->time_updated > params_.time_to_delete_old_obstacles)
            it = ekf_states_.erase(it);
        else
            ++it;
    }
}

void ObstacleTracker::calculateAverageQandR(Eigen::MatrixXd& Q_avg, Eigen::MatrixXd& R_avg)
{
    Q_avg = Eigen::MatrixXd::Zero(9, 9);
    R_avg = Eigen::MatrixXd::Zero(3, 3);

    if (!ekf_states_.empty()) {
        for (const auto& state : ekf_states_) {
            Q_avg += state.Q;
            R_avg += state.R;
        }
        Q_avg /= static_cast<double>(ekf_states_.size());
        R_avg /= static_cast<double>(ekf_states_.size());
    } else {
        Q_avg = Eigen::MatrixXd::Identity(9, 9) * 0.01;
        R_avg = Eigen::MatrixXd::Identity(3, 3) * 0.1;
    }
}

void ObstacleTracker::getCentroidsAndSizes(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<pcl::PointIndices>&      indices,
        std::vector<Eigen::Vector3d>&              centroids,
        std::vector<Eigen::Vector3d>&              bboxes)
{
    centroids.reserve(indices.size());
    bboxes.reserve(indices.size());

    for (const auto& cluster : indices) {
        Eigen::Vector3d min_point = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
        Eigen::Vector3d max_point = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

        for (const auto& index : cluster.indices) {
            const auto&           point = cloud->points[index];
            const Eigen::Vector3d position(point.x, point.y, point.z);
            min_point = min_point.cwiseMin(position);
            max_point = max_point.cwiseMax(position);
        }

        centroids.emplace_back((min_point + max_point) * 0.5);
        const Eigen::Vector3d raw_bbox = max_point - min_point;
        bboxes.emplace_back(Eigen::Vector3d::Constant(raw_bbox.maxCoeff()));
    }
}

void ObstacleTracker::generatePredictions(
        const std::vector<Cluster>& clusters,
        double                      current_time_sec,
        TrackingResult&             result)
{
    int marker_id = 0;

    for (const auto& cluster : clusters) {
        const auto&            ekf              = cluster.ekf_state;
        const auto&            history          = ekf.position_history;
        const Eigen::Vector3d& current_position = cluster.centroid;

        if (static_cast<int>(history.size()) < params_.min_observations_for_prediction)
            continue;

        Eigen::Vector3d velocity(ekf.x[3], ekf.x[4], ekf.x[5]);
        const double    speed = velocity.norm();
        if (speed < params_.cutoff_length_threshold)
            continue;

        if (speed > params_.max_obstacle_velocity)
            velocity *= params_.max_obstacle_velocity / speed;

        PiecewisePolynomial pwp;
        pwp.times = {current_time_sec, current_time_sec + params_.prediction_horizon};
        pwp.coeff_x.push_back({0.0, 0.0, velocity.x(), current_position.x()});
        pwp.coeff_y.push_back({0.0, 0.0, velocity.y(), current_position.y()});
        pwp.coeff_z.push_back({0.0, 0.0, velocity.z(), current_position.z()});

        msg::DynamicObstacleTrajectory trajectory;
        trajectory.header.frame_id = params_.frame_id;
        trajectory.id              = ekf.id;
        trajectory.pos.x           = current_position.x();
        trajectory.pos.y           = current_position.y();
        trajectory.pos.z           = current_position.z();
        trajectory.mode            = "pwp";
        trajectory.bbox            = {
                           static_cast<float>(ekf.bbox.x()), static_cast<float>(ekf.bbox.y()), static_cast<float>(ekf.bbox.z())};
        trajectory.pwp = convertPiecewisePolynomialMessage(pwp);
        trajectory.ekf_cov_p
                = {static_cast<float>(ekf.P(0, 0)), static_cast<float>(ekf.P(1, 1)), static_cast<float>(ekf.P(2, 2))};
        trajectory.ekf_cov_q
                = {static_cast<float>(ekf.Q(0, 0)), static_cast<float>(ekf.Q(1, 1)), static_cast<float>(ekf.Q(2, 2))};
        trajectory.ekf_cov_r
                = {static_cast<float>(ekf.R(0, 0)), static_cast<float>(ekf.R(1, 1)), static_cast<float>(ekf.R(2, 2))};
        trajectory.poly_cov        = {0.0F, 0.0F, 0.0F};
        trajectory.poly_coeffs_x   = {static_cast<float>(velocity.x()), static_cast<float>(current_position.x())};
        trajectory.poly_coeffs_y   = {static_cast<float>(velocity.y()), static_cast<float>(current_position.y())};
        trajectory.poly_coeffs_z   = {static_cast<float>(velocity.z()), static_cast<float>(current_position.z())};
        trajectory.poly_start_time = current_time_sec;
        trajectory.poly_end_time   = current_time_sec + params_.prediction_horizon;
        trajectory.is_agent        = false;
        result.trajectories.push_back(trajectory);

        const double dt_vis       = params_.prediction_dt * 0.25;
        const int    sample_count = static_cast<int>(params_.prediction_horizon / dt_vis) + 1;

        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = params_.frame_id;
        line_marker.ns              = "predicted_obstacle_trajectory";
        line_marker.id              = marker_id++;
        line_marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action          = visualization_msgs::msg::Marker::ADD;
        line_marker.lifetime        = rclcpp::Duration::from_seconds(0.2);
        line_marker.scale.x         = 0.05;
        line_marker.color           = ekf.color;
        line_marker.color.a         = 1.0F;
        line_marker.points.reserve(sample_count);

        for (int step = 0; step < sample_count; ++step) {
            const double              time = step * dt_vis;
            geometry_msgs::msg::Point point;
            point.x = current_position.x() + velocity.x() * time;
            point.y = current_position.y() + velocity.y() * time;
            point.z = current_position.z() + velocity.z() * time;
            line_marker.points.push_back(point);
        }
        result.prediction_markers.markers.push_back(line_marker);
    }
}

void ObstacleTracker::generateBoxMarkers(const std::vector<Cluster>& clusters, TrackingResult& result)
{
    for (const auto& cluster : clusters) {
        const double cx = cluster.centroid.x();
        const double cy = cluster.centroid.y();
        const double cz = cluster.centroid.z();
        const double hx = std::max(cluster.ekf_state.bbox.x(), 0.05) * 0.5;
        const double hy = std::max(cluster.ekf_state.bbox.y(), 0.05) * 0.5;
        const double hz = std::max(cluster.ekf_state.bbox.z(), 0.05) * 0.5;

        visualization_msgs::msg::Marker wire;
        wire.header.frame_id    = params_.frame_id;
        wire.ns                 = "obstacle_bbox";
        wire.id                 = cluster.ekf_state.id;
        wire.type               = visualization_msgs::msg::Marker::LINE_LIST;
        wire.action             = visualization_msgs::msg::Marker::ADD;
        wire.lifetime           = rclcpp::Duration::from_seconds(0.2);
        wire.scale.x            = 0.06;
        wire.color              = cluster.ekf_state.color;
        wire.color.a            = 1.0F;
        wire.pose.orientation.w = 1.0;
        wire.points.reserve(24);

        auto add_edge = [&](double x1, double y1, double z1, double x2, double y2, double z2) {
            geometry_msgs::msg::Point p1;
            geometry_msgs::msg::Point p2;
            p1.x = x1;
            p1.y = y1;
            p1.z = z1;
            p2.x = x2;
            p2.y = y2;
            p2.z = z2;
            wire.points.push_back(p1);
            wire.points.push_back(p2);
        };

        add_edge(cx - hx, cy - hy, cz - hz, cx + hx, cy - hy, cz - hz);
        add_edge(cx + hx, cy - hy, cz - hz, cx + hx, cy + hy, cz - hz);
        add_edge(cx + hx, cy + hy, cz - hz, cx - hx, cy + hy, cz - hz);
        add_edge(cx - hx, cy + hy, cz - hz, cx - hx, cy - hy, cz - hz);
        add_edge(cx - hx, cy - hy, cz + hz, cx + hx, cy - hy, cz + hz);
        add_edge(cx + hx, cy - hy, cz + hz, cx + hx, cy + hy, cz + hz);
        add_edge(cx + hx, cy + hy, cz + hz, cx - hx, cy + hy, cz + hz);
        add_edge(cx - hx, cy + hy, cz + hz, cx - hx, cy - hy, cz + hz);
        add_edge(cx - hx, cy - hy, cz - hz, cx - hx, cy - hy, cz + hz);
        add_edge(cx + hx, cy - hy, cz - hz, cx + hx, cy - hy, cz + hz);
        add_edge(cx + hx, cy + hy, cz - hz, cx + hx, cy + hy, cz + hz);
        add_edge(cx - hx, cy + hy, cz - hz, cx - hx, cy + hy, cz + hz);
        result.bbox_markers.markers.push_back(wire);

        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id    = params_.frame_id;
        sphere.ns                 = "obstacle_centroid";
        sphere.id                 = cluster.ekf_state.id;
        sphere.type               = visualization_msgs::msg::Marker::SPHERE;
        sphere.action             = visualization_msgs::msg::Marker::ADD;
        sphere.lifetime           = rclcpp::Duration::from_seconds(0.2);
        sphere.pose.position.x    = cx;
        sphere.pose.position.y    = cy;
        sphere.pose.position.z    = cz;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x            = 0.2;
        sphere.scale.y            = 0.2;
        sphere.scale.z            = 0.2;
        sphere.color              = cluster.ekf_state.color;
        sphere.color.a            = 1.0F;
        result.bbox_markers.markers.push_back(sphere);
    }
}

std::vector<TrackedObstacle> ObstacleTracker::getTrackedObstacles() const
{
    std::vector<TrackedObstacle> result;
    result.reserve(ekf_states_.size());
    const double velocity_threshold_squared = params_.velocity_threshold * params_.velocity_threshold;

    for (const auto& ekf : ekf_states_) {
        if (ekf.position_history.size() < static_cast<size_t>(params_.min_observations_for_prediction))
            continue;

        if (ekf.x.segment<3>(3).squaredNorm() < velocity_threshold_squared)
            continue;

        result.push_back({ekf.x.head<3>(), ekf.bbox, ekf.id});
    }
    return result;
}

} // namespace dynamic_obstacle_tracker
