#pragma once

#include <Eigen/Dense>
#include <cilantro/utilities/point_cloud.hpp>
#include <cstddef>
#include <dynamic_obstacle_tracker/msg/dynamic_obstacle_trajectory.hpp>
#include <dynamic_obstacle_tracker/msg/piecewise_polynomial3.hpp>
#include <dynamic_obstacle_tracker/msg/polynomial3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace dynamic_obstacle_tracker {

struct PiecewisePolynomial
{
    std::vector<double>                      times;
    std::vector<Eigen::Matrix<double, 4, 1>> coeff_x;
    std::vector<Eigen::Matrix<double, 4, 1>> coeff_y;
    std::vector<Eigen::Matrix<double, 4, 1>> coeff_z;
};

msg::PiecewisePolynomial3 convertPiecewisePolynomialMessage(const PiecewisePolynomial& pwp);

struct EKFState
{
    Eigen::VectorXd          x;
    Eigen::MatrixXd          P;
    Eigen::MatrixXd          Q;
    Eigen::MatrixXd          R;
    double                   time_updated              = 0.0;
    double                   state_time                = 0.0;
    Eigen::Vector3d          bbox                      = Eigen::Vector3d::Zero();
    Eigen::Vector3d          bbox_center_offset        = Eigen::Vector3d::Zero();
    Eigen::Vector3d          last_observed_bbox_center = Eigen::Vector3d::Zero();
    int                      id                        = 0;
    int                      hit_count                 = 1;
    int                      miss_count                = 0;
    bool                     confirmed                 = false;
    std_msgs::msg::ColorRGBA color;

    EKFState() = default;
    EKFState(
            int                    state_size,
            const Eigen::MatrixXd& Q_init,
            const Eigen::MatrixXd& R_init,
            double                 time,
            const Eigen::Vector3d& bbox_init,
            int                    id_val);

    std::vector<std::pair<double, Eigen::Vector3d>> position_history;
};

struct ClusterDetection
{
    Eigen::Vector3d centroid            = Eigen::Vector3d::Zero();
    Eigen::Vector3d bbox                = Eigen::Vector3d::Zero();
    Eigen::Vector3d bbox_center_offset  = Eigen::Vector3d::Zero();
    Eigen::Matrix3d centroid_covariance = Eigen::Matrix3d::Identity();
    std::size_t     point_count         = 0;
};

struct ObstacleTrackerParams
{
    bool        use_adaptive_kf                 = true;
    double      adaptive_kf_alpha               = 0.90;
    double      tracking_voxel_size             = 0.15;
    double      cluster_tolerance               = 1.0;
    int         min_cluster_size                = 5;
    int         max_cluster_size                = 2000;
    double      prediction_horizon              = 1.0;
    double      time_to_delete_old_obstacles    = 0.5;
    double      cluster_bbox_cutoff_size        = 2.0;
    double      centroid_measurement_noise      = 0.10;
    double      mahalanobis_gate_squared        = 11.345;
    double      maximum_association_distance    = 1.0;
    double      velocity_threshold              = 0.8;
    double      cutoff_length_threshold         = 0.1;
    int         max_history_size                = 30;
    int         min_observations_for_prediction = 5;
    int         confirmation_hits               = 3;
    int         tentative_max_missed_scans      = 1;
    double      max_obstacle_velocity           = 2.0;
    double      max_obstacle_acceleration       = 2.0;
    std::string frame_id;
};

struct TrackedObstacle
{
    Eigen::Vector3d position;
    Eigen::Vector3d bbox;
    int             id;
};

struct TrackingResult
{
    struct Timings
    {
        double state_cleanup_ms = 0.0;
        double finite_filter_ms = 0.0;
        double downsampling_ms  = 0.0;
        double clustering_ms    = 0.0;
        double feature_ms       = 0.0;
        double kf_prediction_ms = 0.0;
        double association_ms   = 0.0;
        double state_update_ms  = 0.0;
        double bbox_markers_ms  = 0.0;
        double prediction_ms    = 0.0;
        double total_ms         = 0.0;
    };

    std::vector<msg::DynamicObstacleTrajectory> trajectories;
    visualization_msgs::msg::MarkerArray        bbox_markers;
    visualization_msgs::msg::MarkerArray        prediction_markers;
    Timings                                     timings;
    std::size_t                                 input_point_count       = 0;
    std::size_t                                 finite_point_count      = 0;
    std::size_t                                 downsampled_point_count = 0;
    std::size_t                                 candidate_cluster_count = 0;
    std::size_t                                 accepted_cluster_count  = 0;
    std::size_t                                 active_track_count      = 0;
    std::size_t                                 confirmed_track_count   = 0;
    std::size_t                                 coasting_track_count    = 0;
    std::size_t                                 matched_track_count     = 0;
    std::size_t                                 new_track_count         = 0;
};

class ObstacleTracker
{
  public:
    explicit ObstacleTracker(const ObstacleTrackerParams& params, rclcpp::Logger logger);

    TrackingResult update(const cilantro::PointCloud3f& dynamic_cloud, double current_time_sec);

    void setFrameId(std::string frame_id);

    std::vector<TrackedObstacle> getTrackedObstacles() const;

  private:
    static void ekfPredict(EKFState& state, double dt);
    static bool aekfUpdate(
            EKFState&              state,
            const Eigen::Vector3d& z,
            const Eigen::Matrix3d& measurement_covariance,
            double                 alpha,
            double                 time_updated,
            const Eigen::Vector3d& bbox,
            const Eigen::Vector3d& bbox_center_offset,
            bool                   use_adaptive);
    static double          mahalanobisDistanceSquared(const EKFState& state, const ClusterDetection& detection);
    static Eigen::Vector3d outputPosition(const EKFState& state);

    void                          deleteOldStates(double current_time);
    void                          predictStates(double current_time);
    void                          constrainMotion(EKFState& state) const;
    void                          calculateAverageQandR(Eigen::MatrixXd& Q_avg, Eigen::MatrixXd& R_avg);
    std::vector<ClusterDetection> extractDetections(
            const Eigen::Matrix<float, 3, Eigen::Dynamic>& cloud,
            const std::vector<std::vector<std::size_t>>&   indices) const;
    void appendToHistory(EKFState& state, double time, const Eigen::Vector3d& pos);
    void generatePredictions(double current_time_sec, TrackingResult& result);
    void generateBoxMarkers(TrackingResult& result);

    ObstacleTrackerParams   params_;
    rclcpp::Logger          logger_;
    std::vector<EKFState>   ekf_states_;
    std::unordered_set<int> previous_bbox_marker_ids_;
    std::unordered_set<int> previous_prediction_marker_ids_;
    int                     next_ekf_id_ = 0;
};

} // namespace dynamic_obstacle_tracker
