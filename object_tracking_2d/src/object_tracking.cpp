#include "object_tracking_2d/object_tracking.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace state_estimation
{


ObjectTracking::ObjectTracking() : Node("object_tracking_2d"), laser_scan_filter_(30.0, 0.06, 180.0, -180.0, 3)
{   
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    this->declare_parameter("lidar_frame", "lidar_frame");
    this->declare_parameter("odom_frame", "odom_frame");
    lidar_frame_ = this->get_parameter("lidar_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();

    /**
     * @brief param wrt LaserScan Pre-Process 
     * 
     */
    auto range_max_config = rcl_interfaces::msg::ParameterDescriptor{};
    range_max_config.description = "max distance from lasescan frame to object to be detected";
    this->declare_parameter("range_max", 30.0, range_max_config);
    float range_max = this->get_parameter("range_max").as_double();
    
    auto range_min_config = rcl_interfaces::msg::ParameterDescriptor{};
    range_min_config.description = "min distance from sensor frame to object to be detected";
    this->declare_parameter("range_min", 0.06, range_min_config);
    float range_min = this->get_parameter("range_min").as_double();
    
    auto angle_max_config = rcl_interfaces::msg::ParameterDescriptor{};
    angle_max_config.description = "max angle in a horizontal direction to object to be detected";
    this->declare_parameter("angle_max", 6.2831852, angle_max_config);
    float angle_max = this->get_parameter("angle_max").as_double();
    
    auto angle_min_config = rcl_interfaces::msg::ParameterDescriptor{};
    angle_min_config.description = "min angle in a horizontal direction to object to be detected";
    this->declare_parameter("angle_min", 0.0, angle_min_config);
    float angle_min = this->get_parameter("angle_min").as_double();
    
    auto ma_interval_config = rcl_interfaces::msg::ParameterDescriptor{};
    ma_interval_config.description = "ave of ma_interval samples for moving average pre-process";
    ma_interval_config.additional_constraints = "resolution of laserscan, ma_interval and cluster_points highly depend on each other";
    this->declare_parameter("ma_interval", 100, ma_interval_config);
    int ma_interval = this->get_parameter("ma_interval").as_int();

    // laser scan filter for pre-process
    laser_scan_filter_ = LaserScanFilter(
        range_max, 
        range_min, 
        angle_max, 
        angle_min, 
        ma_interval
    );

    /**
     * @brief param wrt clustering objects
     * 
     */
    auto freq_config = rcl_interfaces::msg::ParameterDescriptor{};
    freq_config.description = "detection callback frequency";
    freq_config.additional_constraints = "has to be less than lidar scan frequency";
    this->declare_parameter("frequency", 20, freq_config);

    auto cluster_points_config = rcl_interfaces::msg::ParameterDescriptor{};
    cluster_points_config.description = "clustering an object";
    cluster_points_config.additional_constraints = "resolution of laserscan, ma_interval and cluster_points highly depend on each other";
    this->declare_parameter("cluster_points", 30, cluster_points_config);
    cluster_points_ = this->get_parameter("cluster_points").as_int();
    object_center_ = cluster_points_ / 2;

    auto cluster_angle_tolerance_config = rcl_interfaces::msg::ParameterDescriptor{};
    cluster_angle_tolerance_config.description = "gap between two objects in a horizontal direction";
    cluster_angle_tolerance_config.additional_constraints = "highly depending on situation (e.g. large number under the situation that there are many objects)";
    this->declare_parameter("cluster_angle_tolerance", 0.5, cluster_angle_tolerance_config);
    cluster_angle_tolerance_ = this->get_parameter("cluster_angle_tolerance").as_double();

    auto cluster_dist_tolerance_config = rcl_interfaces::msg::ParameterDescriptor{};
    cluster_dist_tolerance_config.description = "gap between two objects in a radial direction";
    cluster_dist_tolerance_config.additional_constraints = "highly depending on situation (e.g. large number under the situation that there are many objects)";
    this->declare_parameter("cluster_dist_tolerance", 0.5, cluster_dist_tolerance_config);
    cluster_dist_tolerance_ = this->get_parameter("cluster_dist_tolerance").as_double();

    RCLCPP_WARN(get_logger(), "Laser Scan Filter Parameter");
    RCLCPP_INFO(get_logger(), "\tlidar_frame: %s", lidar_frame_.c_str());
    RCLCPP_INFO(get_logger(), "\todom_frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(get_logger(), "\trange_max: %lf", range_max);
    RCLCPP_INFO(get_logger(), "\trange_min: %lf", range_min);
    RCLCPP_INFO(get_logger(), "\tangle_max: %lf", angle_max);
    RCLCPP_INFO(get_logger(), "\tangle_min: %lf", angle_min);
    RCLCPP_INFO(get_logger(), "\tma_interval: %d", ma_interval);
    RCLCPP_INFO(get_logger(), "\tcluster_points: %d", cluster_points_);
    RCLCPP_INFO(get_logger(), "\tcluster_angle_tolerance: %lf", cluster_angle_tolerance_);
    RCLCPP_INFO(get_logger(), "\tcluster_dist_tolerance: %lf", cluster_dist_tolerance_);
    
    // initialize system to be tracked by kalman filter
    initializeKFSystem();

    // lidar qos
    rmw_qos_profile_t custom_qos_profile_lidar = rmw_qos_profile_default;
    custom_qos_profile_lidar.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos_profile_lidar.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    custom_qos_profile_lidar.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE;
    rclcpp::QoS qos_profile_lidar(rclcpp::KeepLast(1), custom_qos_profile_lidar);
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos_profile_lidar, std::bind(&ObjectTracking::LaserScanCallback, this, std::placeholders::_1));

    // tracks with header topic
    rmw_qos_profile_t custom_qos_profile_tracking = rmw_qos_profile_default;
    custom_qos_profile_tracking.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos_profile_tracking.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    custom_qos_profile_tracking.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE;
    rclcpp::QoS qos_profile_tracking(rclcpp::KeepLast(5), custom_qos_profile_tracking);
    object_publisher_ = this->create_publisher<track_msgs::msg::TrackArray>(
        "tracks", qos_profile_tracking
    );

    // visualizer qos
    rmw_qos_profile_t custom_qos_profile_visualization = rmw_qos_profile_default;
    custom_qos_profile_visualization.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos_profile_visualization.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    custom_qos_profile_visualization.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE;
    rclcpp::QoS qos_profile_visualization(rclcpp::KeepLast(5), custom_qos_profile_visualization);
    static_obs_posi_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "static_objects/position", qos_profile_visualization);
    static_obs_posi_cov_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "static_objects/position/covariance", qos_profile_visualization);
    static_obs_vel_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "static_objects/velocity", qos_profile_visualization);
    static_obs_traj_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "static_objects/trajectory", qos_profile_visualization);
    dynamic_obs_posi_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "dynamic_objects/position", qos_profile_visualization);
    dynamic_obs_next_posi_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "dynamic_objects/next_position", qos_profile_visualization);
    dynamic_obs_posi_cov_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "dynamic_objects/position/covariance", qos_profile_visualization);
    dynamic_obs_vel_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "dynamic_objects/velocity", qos_profile_visualization);
    dynamic_obs_traj_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "dynamic_objects/trajectory", qos_profile_visualization);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000/this->get_parameter("frequency").as_int()),
        std::bind(&ObjectTracking::DetectionCallback, this)
    );

    prev_stamp_ = this->get_clock()->now();

    RCLCPP_INFO(get_logger(), "%s start!!\n", this->get_name());
}
ObjectTracking::~ObjectTracking(){}

void ObjectTracking::initializeKFSystem()
{
    /**
     * @brief Kalmn Filter param
     * 
     */
    // process noise param def
    auto kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "variance wrt position noise";
    this->declare_parameter("position_noise_variance", 1.0, kf_param_config);
    float position_noise_variance = this->get_parameter("position_noise_variance").as_double();
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "variance wrt velocity noise";
    this->declare_parameter("velocity_noise_variance", 1.0, kf_param_config);
    float velocity_noise_variance = this->get_parameter("velocity_noise_variance").as_double();
    // observation noise param def
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "variance wrt observation noise";
    this->declare_parameter("observation_noise_variance", 1.0, kf_param_config);
    float observation_noise_variance = this->get_parameter("observation_noise_variance").as_double();
    // initial covariance param def
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "variance wrt position error";
    this->declare_parameter("position_error_variance", 1.0, kf_param_config);
    float position_error_variance = this->get_parameter("position_error_variance").as_double();
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "variance wrt velocity error";
    this->declare_parameter("velocity_error_variance", 1.0, kf_param_config);
    float velocity_error_variance = this->get_parameter("velocity_error_variance").as_double();

    /**
     * @brief constant acceleration model
     * 
     */
    // process noise
    Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity(4, 4);
    process_noise.block(0,0, 2,2) *= position_noise_variance;
    process_noise.block(2,2, 2,2) *= velocity_noise_variance;
    // observation noise
    Eigen::MatrixXf observation_noise = Eigen::MatrixXf::Identity(2, 2)*observation_noise_variance;
    // system model
    Eigen::Matrix<float, 4, 4> system_model;
    system_model.setIdentity();
    // control model
    Eigen::Matrix<float, 4, 4> control_model;
    control_model.setZero();
    // observation model
    Eigen::Matrix<float, 2, 4> observation_model;
    observation_model.setZero();
    observation_model.block(0,0, 2,2) = Eigen::Matrix2f::Identity();
    // initialize system
    system_.reset(
        new System(
            process_noise, 
            observation_noise, 
            system_model, 
            control_model, 
            observation_model
        )
    );
    // initial covariance
    initial_covariance_ = Eigen::MatrixXf::Identity(4, 4);
    initial_covariance_.block(0,0, 2,2) *= position_error_variance;
    initial_covariance_.block(2,2, 2,2) *= velocity_error_variance;

    RCLCPP_WARN(get_logger(), "Tracking Parameter");
    RCLCPP_INFO(get_logger(), "\tprocess noise:");
    for (int i=0; i<4; i++)
    {
        RCLCPP_INFO(get_logger(), "\t\t[%.3lf, %.3lf, %.3lf, %.3lf]", 
            process_noise(i, 0), process_noise(i, 1), process_noise(i, 2), process_noise(i, 3));
    }
    RCLCPP_INFO(get_logger(), "\tobservation noise:");
    for (int i=0; i<2; i++)
    {
        RCLCPP_INFO(get_logger(), "\t\t[%.3lf, %.3lf]", 
            observation_noise(i, 0), observation_noise(i, 1));
    }
    RCLCPP_INFO(get_logger(), "\tinitial covariance:");
    for (int i=0; i<4; i++)
    {
        RCLCPP_INFO(get_logger(), "\t\t[%.3lf, %.3lf, %.3lf, %.3lf]", 
            initial_covariance_(i, 0), initial_covariance_(i, 1), initial_covariance_(i, 2), initial_covariance_(i, 3));
    }

    // threshold parameter def for tracking
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "frame out if object has not been corrected for over this";
    this->declare_parameter("frames_limit", 1, kf_param_config);
    frames_limit_ = this->get_parameter("frames_limit").as_int();
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "untracked if object velocity is higher than this";
    this->declare_parameter("velocity_limit", 1.0, kf_param_config);
    velocity_limit_ = this->get_parameter("velocity_limit").as_double();
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "matching with observation within this distance";
    this->declare_parameter("matching_dist", 1.0, kf_param_config);
    matching_dist_ = this->get_parameter("matching_dist").as_double();
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "outlier if out of range of this sigma thresh";
    this->declare_parameter("maha_dist_sigma", 1.0, kf_param_config);
    maha_dist_sigma_ = this->get_parameter("maha_dist_sigma").as_double();
    maha_dist_sigma_2_ = std::pow(maha_dist_sigma_, 2.0);
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "velocity criteria to determine dynamic or static object";
    this->declare_parameter("velocity_sta2dyn", 1.0, kf_param_config);
    velocity_sta2dyn_ = this->get_parameter("velocity_sta2dyn").as_double();
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "the number of frames to be changed from static to dynamic";
    this->declare_parameter("frames_sta2dyn", 20, kf_param_config);
    frames_sta2dyn_ = this->get_parameter("frames_sta2dyn").as_int();
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "the number of frames to be changed from dynamic to static";
    this->declare_parameter("frames_dyn2sta", 20, kf_param_config);
    frames_dyn2sta_ = this->get_parameter("frames_dyn2sta").as_int();
    kf_param_config = rcl_interfaces::msg::ParameterDescriptor{};
    kf_param_config.description = "person position in this time step";
    this->declare_parameter("time_horizon", 1.0, kf_param_config);
    time_horizon_ = this->get_parameter("time_horizon").as_double();
    this->declare_parameter("max_track_num", 1000);
    max_track_num_ = this->get_parameter("max_track_num").as_int();

    std::random_device rd;
    gen_= std::mt19937(rd());
    distr_ = std::uniform_int_distribution<>(1, max_track_num_);

    RCLCPP_INFO(get_logger(), "\tframes_limit: %d", frames_limit_);
    RCLCPP_INFO(get_logger(), "\tvelocity_limit: %.3lf", velocity_limit_);
    RCLCPP_INFO(get_logger(), "\tmatching_dist: %.3lf", matching_dist_);
    RCLCPP_INFO(get_logger(), "\tmaha_dist_sigma: %.3lf", maha_dist_sigma_);
    RCLCPP_INFO(get_logger(), "\tvelocity_sta2dyn: %.3lf", velocity_sta2dyn_);
    RCLCPP_INFO(get_logger(), "\tframes_dyn2sta: %d", frames_dyn2sta_);
    RCLCPP_INFO(get_logger(), "\tframes_sta2dyn: %d", frames_sta2dyn_);
    RCLCPP_INFO(get_logger(), "\tfuture_time: %.3lf", time_horizon_);
    RCLCPP_INFO(get_logger(), "\tmax_track_num: %d", max_track_num_);

    this->declare_parameter("visualize_covariance", false);
    this->declare_parameter("visualize_trajectory", false);
    this->declare_parameter("visualize_next_position", false);
    visualize_covariance_ = this->get_parameter("visualize_covariance").as_bool();
    visualize_trajectory_ = this->get_parameter("visualize_trajectory").as_bool();
    visualize_next_position_ = this->get_parameter("visualize_next_position").as_bool();
    RCLCPP_INFO(get_logger(), "\tvisualize_covariance: %d", visualize_covariance_);
    RCLCPP_INFO(get_logger(), "\tvisualize_trajectory: %d", visualize_trajectory_);
    RCLCPP_INFO(get_logger(), "\tvisualize_next_position: %d", visualize_next_position_);
}

void ObjectTracking::LaserScanCallback(const sensor_msgs::msg::LaserScan &msg){scan_ = msg;}

void ObjectTracking::DetectionCallback()
{
    // laser scan pre-process
    std::vector<LaserScanFilter::PreProcScan> preproc_scans;
    laser_scan_filter_.PreProc(scan_, preproc_scans);

    // compute local minimums, means detect object candidates
    std::vector<LaserScanFilter::PreProcScan> local_minimums = computeLocalMinimums(scan_, preproc_scans);
    RCLCPP_INFO(get_logger(), "Object candidates: %d", local_minimums.size());

    // get transform
    rclcpp::Time cur_stamp = this->get_clock()->now();
    tf2::Transform odom2lidar;
    try 
    {
        geometry_msgs::msg::TransformStamped lidar2odom_msg = 
            tf_buffer_->lookupTransform(odom_frame_, lidar_frame_, cur_stamp, tf2::durationFromSec(0.1));
        tf2::fromMsg(lidar2odom_msg.transform, odom2lidar);
        RCLCPP_DEBUG(get_logger(), "get transform from [%s] to [%s]", lidar_frame_.c_str(), odom_frame_.c_str());
    }
    catch (tf2::TransformException &ex) 
    {
        RCLCPP_DEBUG(this->get_logger(), "%s", ex.what());
        return;
    }

    // transform from lidar_frame to odom_frame
    std::vector<Eigen::Vector2f> observations;
    for (int i=0; i<local_minimums.size(); i++)
    {
        tf2::Transform lidar2target;
        lidar2target.setOrigin(
            tf2::Vector3(
                -local_minimums[i].range*cosf(local_minimums[i].angle), 
                local_minimums[i].range*sinf(-local_minimums[i].angle), 
                0.0
            )
        );

        tf2::Transform odom2target = odom2lidar * lidar2target;

        Eigen::Vector2f observation;
        observation[0] = odom2target.getOrigin().getX();
        observation[1] = odom2target.getOrigin().getY();
        observations.push_back(observation);
    }
    
    double cur_time = cur_stamp.nanoseconds()/1000000000.0;
    std::vector<int> unmatched_observation_id;
    if (objects_.size() > 0)
    {
        /**
        * @brief Kalman filter prediction
        * if any tracked objects, kalman filter prediction first.
        * hugarian solver solves assignment problem depending on cost definition.
        */
        RCLCPP_INFO(get_logger(), "KF Prediction");
        Eigen::VectorXf control = Eigen::VectorXf::Zero(4);
        vector<vector<double>> cost;
        for (int i=0; i<objects_.size(); i++)
        {
            objects_.at(i)->predict(
                (cur_stamp-prev_stamp_).nanoseconds()/1000000000.0, 
                control
            );
            RCLCPP_WARN(get_logger(), "\tid%d predicted. vel %.3lf", objects_.at(i)->id(), objects_.at(i)->velocity().norm());
            

            // compute cost
            int len_traj = objects_.at(i)->trajectory().size();
            std::vector<double> cost_row;
            for (int j=0; j<observations.size(); j++)
            {
                double eucl_dist = objects_.at(i)->computeEuclDistance(observations[j]);
                double squared_maha_dist = objects_.at(i)->computeSquaredMahaDistance(observations[j]);
                double weight = (
                    (squared_maha_dist < maha_dist_sigma_2_ ? 1.0 : 100.0) // mahalanobis gate
                    * (len_traj > 30 ? 1.0 : 2.0)  // existing track with longer trajectory should be easilr associated
                    // * (objects_.at(i)->isStillTentative() ? 2.0 : 1.0)
                );
                cost_row.push_back(eucl_dist * weight);
            }
            cost.push_back(cost_row);
        }
        HungarianAlgorithm assignment_solver;
        vector<int> assignment;
        double cost_metric = assignment_solver.Solve(cost, assignment);


        /**
        * @brief Kalman filter correction
        * according to assignment result, existing tracked objects are associated with the observation.
        * observations that were not associated with any tracks, they are assigned to unmatch observation list.
        */
        RCLCPP_INFO(get_logger(), "KF Correction");
        for (int i=0; i<observations.size(); i++)
        {
            auto itr = std::find(assignment.begin(), assignment.end(), i);
            int index = std::distance(assignment.begin(), itr);
            try{
                if (objects_.at(index)->computeEuclDistance(observations.at(i))<matching_dist_)
                {
                    objects_.at(index)->correct(
                        observations.at(i), 
                        cur_time, 
                        frames_sta2dyn_, frames_dyn2sta_,  
                        velocity_sta2dyn_
                    );
                    RCLCPP_WARN(get_logger(), "\tid%d corrected. vel %.3lf", objects_.at(index)->id(), objects_.at(index)->velocity().norm());
                }
                
            }
            catch (std::out_of_range& ex) {unmatched_observation_id.push_back(i);}
        }

        RCLCPP_INFO(get_logger(), "Delete objects");
        for (int i=0; i<objects_.size();)
        {
            if (objects_.at(i)->isStillTentative() ||
                objects_.at(i)->isUncorrectedOverThan(frames_limit_) || 
                objects_.at(i)->isHigherSpeedThan(velocity_limit_) )
            {
                RCLCPP_WARN(get_logger(), "\tid%d deleted.", objects_.at(i)->id());
                existing_ids_.erase(objects_.at(i)->id());
                objects_.erase(objects_.begin()+i);
            }
            else i++;
        }

    }
    else
    {
        for (int i=0; i<observations.size(); i++) unmatched_observation_id.push_back(i);
    }
        

    /**
     * @brief new objects join
     * unmatched observations are assigned to new objects to be tracked
     */
    RCLCPP_INFO(get_logger(), "New assign");
    for (int id : unmatched_observation_id)
    {
        try
        {
            int new_id;
            while (true)
            {
                new_id = distr_(gen_);
                if (existing_ids_.find(new_id) == existing_ids_.end())
                {
                    existing_ids_.insert(new_id);
                    break;
                }
            }
            Eigen::VectorXf observation(4);
            observation << 
                observations.at(id)[0], 
                observations.at(id)[1], 
                0.0, 
                0.0;
            objects_.push_back(std::make_shared<Object>(
                cur_time, system_, observation, initial_covariance_, new_id));
            RCLCPP_WARN(get_logger(), "\tid%d join.", new_id);
        }
        catch (std::out_of_range& ex) {continue;}
    }


    publishObjects(cur_stamp);
    
    prev_stamp_ = cur_stamp;
}

std::vector<LaserScanFilter::PreProcScan> ObjectTracking::computeLocalMinimums(
    const sensor_msgs::msg::LaserScan &original_scans, 
    const std::vector<LaserScanFilter::PreProcScan> &preproc_scans
) const
{
    // detect candidates
    std::vector<LaserScanFilter::PreProcScan> candidates;
    int num_preproc_scans = preproc_scans.size();
    for (int i=object_center_; i<num_preproc_scans-object_center_; i++)
    {
        if (preproc_scans[i-object_center_].range > preproc_scans[i].range // explore local minimum from left
            && preproc_scans[i].range < preproc_scans[i+object_center_].range) // explore local minimum from right
        {
            LaserScanFilter::PreProcScan min_scan=preproc_scans[i];
            for (int j=i+1; j<i+object_center_; j++)
            {
                if (preproc_scans[j].range < min_scan.range)
                {
                    min_scan = preproc_scans[j]; // udpate local minimum
                }
            }
            // if (min_scan.range > 10.0) continue;
            i+=object_center_;

            // filter candidate with tolerance in angle and dist
            if (candidates.size() == 0) candidates.push_back(min_scan);
            else 
            {
                LaserScanFilter::PreProcScan& tail = candidates.back();
                double tx = tail.range*cosf(tail.angle);
                double ty = tail.range*sinf(tail.angle);
                double mx = min_scan.range*cosf(min_scan.angle);
                double my = min_scan.range*sinf(min_scan.angle);
                if ((std::pow(tx-mx, 2.0)+std::pow(ty-my, 2.0) > std::pow(cluster_dist_tolerance_, 2.0) 
                    || fabs(tail.angle - min_scan.angle) > cluster_angle_tolerance_))
                {
                    candidates.push_back(min_scan);
                }
                else if (tail.range > min_scan.range)
                {
                    candidates.pop_back();
                    candidates.push_back(min_scan);
                }
            }
        }
    }
    // delete duplicate
    if (candidates.size() > 1)
    {
        while (true)
        {
            LaserScanFilter::PreProcScan &tail = candidates.back(); 
            double tx = tail.range*cosf(tail.angle);
            double ty = tail.range*sinf(tail.angle);
            double mx = candidates[0].range*cosf(candidates[0].angle);
            double my = candidates[0].range*sinf(candidates[0].angle);
            if (std::pow(tx-mx, 2.0)+std::pow(ty-my, 2.0) < std::pow(cluster_dist_tolerance_, 2.0)
                && fabs(fabs(tail.angle - candidates[0].angle)-6.28) < cluster_angle_tolerance_)
            {
                candidates.pop_back();
            }
            else break;
        }
    }

    return candidates;
}

void ObjectTracking::publishObjects(const rclcpp::Time &stamp) const
{
    // color
    std_msgs::msg::ColorRGBA light_green, dark_green;
    light_green.r = 0.3;
    light_green.g = 1.0;
    light_green.b = 0.3;
    dark_green.r = 0.0;
    dark_green.g = 0.5;
    dark_green.b = 0.5;

    visualization_msgs::msg::MarkerArray markers, markers_vel; // static osb position & velocity marker array
    visualization_msgs::msg::MarkerArray dyn_markers, dyn_markers_vel; // dynamic obs position & velocity marker array
    visualization_msgs::msg::MarkerArray dyn_next_markers; // dynamic obs next position marker array
    visualization_msgs::msg::MarkerArray markers_pos_cov, markers_traj; // static obs position covariance & trajectory marker array
    visualization_msgs::msg::MarkerArray dyn_markers_pos_cov, dyn_markers_traj; // dynamic obs position covariance & trajectory marker array
    
    track_msgs::msg::TrackArray tracks;
    tracks.header.stamp = stamp;
    tracks.header.frame_id = odom_frame_;
    // position marker
    visualization_msgs::msg::Marker marker;
    marker.header = tracks.header;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.z = 0.2;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.4;
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    marker.lifetime = rclcpp::Duration::from_seconds(0.07);

    // duplicate for text visualization
    visualization_msgs::msg::Marker marker_id_text = marker;
    marker_id_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker_id_text.pose.position.z = 0.5;
    marker_id_text.color.a = 1.0; // black
    marker_id_text.color.r = 0.0;
    marker_id_text.color.g = 0.0;
    marker_id_text.color.b = 0.0;

    // duplicate velicity marker
    visualization_msgs::msg::Marker marker_vel = marker;
    marker_vel.type = visualization_msgs::msg::Marker::ARROW;
    marker_vel.scale.x = 0.0;
    marker_vel.scale.y = 0.1;
    marker_vel.scale.z = 0.1;

    // duplicate for covariance 
    visualization_msgs::msg::Marker marker_pos_cov;
    marker_pos_cov = marker;

    // duplicate for trajectory
    visualization_msgs::msg::Marker marker_traj;
    marker_traj = marker;
    marker_traj.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_traj.scale.x = 0.03;

    int num_objects = objects_.size();
    for (int i=0; i<num_objects; i++)
    {
        if (objects_[i]->isStillTentative()) continue;

        track_msgs::msg::Track track = objects_[i]->toTrackMsg();
        tracks.tracks.push_back(track);

        if (objects_[i]->isDynamic())
        {
            // add position & text marker
            marker.id = track.id*2; // position marker array contains position & id visualization 
            marker.pose.position = track.position;
            marker.color = dark_green;
            marker.color.a = 1.0;
            dyn_markers.markers.push_back(marker);
            marker_id_text.id = marker.id+1; // position marker array contains position & id visualization 
            marker_id_text.pose.position.x = track.position.x;
            marker_id_text.pose.position.y = track.position.y;
            marker_id_text.text = std::to_string(objects_[i]->id());
            dyn_markers.markers.push_back(marker_id_text);

            // add velocity marker
            marker_vel.id = marker.id;
            marker_vel.pose.position = track.position;
            tf2::Quaternion q;
            q.setRPY(0, 0, atan2(track.velocity.y, track.velocity.x));
            marker_vel.pose.orientation = tf2::toMsg(q);
            marker_vel.scale.x = std::max(static_cast<double>(objects_[i]->velocity().norm()), 0.0000001);
            marker_vel.color = dark_green;
            marker_vel.color.a = 1.0;
            dyn_markers_vel.markers.push_back(marker_vel);

            // add next position
            if (visualize_next_position_)
            {
                marker.color.a = 0.5;
                marker.pose.position.x += time_horizon_ * track.velocity.x;
                marker.pose.position.y += time_horizon_ * track.velocity.y;
                dyn_next_markers.markers.push_back(marker);
                dyn_next_markers.markers.push_back(marker_id_text);
            }

            // add covariance marker
            if (visualize_covariance_)
            {
                Eigen::Matrix2f covariance = objects_[i]->covariance().block(0,0, 2,2); // get covariance wrt position
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance);
                Eigen::Vector2f eigenvalues = eigen_solver.eigenvalues();
                Eigen::Matrix2f eigenvectors = eigen_solver.eigenvectors();
                Eigen::Vector2f scale = (maha_dist_sigma_ * 2 * eigenvalues).array().sqrt();

                marker_pos_cov.id = track.id;
                marker_pos_cov.scale.x = scale[0];
                marker_pos_cov.scale.y = scale[1];
                marker_pos_cov.scale.z = 0.01;
                marker_pos_cov.pose.position = track.position;
                marker_pos_cov.pose.position.z = 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, atan2(eigenvectors(1, 0), eigenvectors(0, 0)));
                marker_pos_cov.pose.orientation = tf2::toMsg(q);
                marker_pos_cov.color = dark_green;
                marker_pos_cov.color.a = 0.2;
                dyn_markers_pos_cov.markers.push_back(marker_pos_cov);
            }

            // add trajectory marker
            if (visualize_trajectory_)
            {
                marker_traj.id = track.id;
                marker_traj.points = objects_[i]->trajectory();
                marker_traj.color = dark_green;
                marker_traj.color.a = 1.0;
                dyn_markers_traj.markers.push_back(marker_traj);
            }
        }
        else // if static object
        {
            // add position & text visualization marker
            marker.id = track.id*2;
            marker.pose.position = track.position;
            marker.color = light_green;
            marker.color.a = 1.0;
            markers.markers.push_back(marker);
            marker_id_text.id = marker.id+1;
            marker_id_text.pose.position.x = track.position.x;
            marker_id_text.pose.position.y = track.position.y;
            marker_id_text.text = std::to_string(track.id);
            markers.markers.push_back(marker_id_text);

            // add velocity marker
            marker_vel.id = marker.id;
            marker_vel.pose.position = track.position;
            tf2::Quaternion q;
            q.setRPY(0, 0, atan2(track.velocity.y, track.velocity.x));
            marker_vel.pose.orientation = tf2::toMsg(q);
            marker_vel.scale.x = std::max(static_cast<double>(objects_[i]->velocity().norm()), 0.0000001);
            marker_vel.color = light_green;
            marker_vel.color.a = 1.0;
            markers_vel.markers.push_back(marker_vel);
            
            // add covariance marker
            if (visualize_covariance_)
            {
                Eigen::Matrix2f covariance = objects_[i]->covariance().block(0,0, 2,2); // get covariance wrt position
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance);
                Eigen::Vector2f eigenvalues = eigen_solver.eigenvalues();
                Eigen::Matrix2f eigenvectors = eigen_solver.eigenvectors();
                Eigen::Vector2f scale = (maha_dist_sigma_ * 2 * eigenvalues).array().sqrt();

                marker_pos_cov.id = track.id;
                marker_pos_cov.scale.x = scale[0];
                marker_pos_cov.scale.y = scale[1];
                marker_pos_cov.scale.z = 0.01;
                marker_pos_cov.pose.position = marker.pose.position;
                marker_pos_cov.pose.position.z = 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, atan2(eigenvectors(1, 0), eigenvectors(0, 0)));
                marker_pos_cov.pose.orientation = tf2::toMsg(q);
                marker_pos_cov.color = light_green;
                marker_pos_cov.color.a = 0.2;
                markers_pos_cov.markers.push_back(marker_pos_cov);
            }

            // add trajectory marker
            if (visualize_trajectory_)
            {
                marker_traj.id = track.id;
                marker_traj.points = objects_[i]->trajectory();
                marker_traj.color = light_green;
                marker_traj.color.a = 1.0;
                markers_traj.markers.push_back(marker_traj);
            }
        }
    }
    object_publisher_->publish(tracks);
    static_obs_posi_marker_publisher_->publish(markers);
    static_obs_vel_marker_publisher_->publish(markers_vel);
    dynamic_obs_posi_marker_publisher_->publish(dyn_markers);
    dynamic_obs_vel_marker_publisher_->publish(dyn_markers_vel);
    if (visualize_next_position_) dynamic_obs_next_posi_marker_publisher_->publish(dyn_next_markers);
    if (visualize_covariance_) 
    {
        static_obs_posi_cov_marker_publisher_->publish(markers_pos_cov);
        dynamic_obs_posi_cov_marker_publisher_->publish(dyn_markers_pos_cov);
    }
    if (visualize_trajectory_) 
    {
        static_obs_traj_marker_publisher_->publish(markers_traj);
        dynamic_obs_traj_marker_publisher_->publish(dyn_markers_traj);
    }
}

} // namespace state_estimation