#include "object_tracking_2d/object_tracking.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ros2_cpp_utils/utils.hpp" // rosparam getter, qos getter
#include "ros2_cpp_utils/rviz.hpp" // visualization utils

namespace state_estimation
{


ObjectTracking::ObjectTracking() : Node("object_tracking_2d"), laser_scan_filter_(30.0, 0.06, 180.0, -180.0, 3)
{   
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    lidar_frame_ = ros2_cpp_utils::utils::getRosParam<std::string>(this, "lidar_frame", "lidar_frame");
    odom_frame_ = ros2_cpp_utils::utils::getRosParam<std::string>(this, "odom_frame", "odom_frame");
    
    initializeLaserScanFilter(); 
    initializeKFSystem();

    // lidar qos
    rclcpp::QoS qos_profile_lidar = ros2_cpp_utils::utils::getQoS(this, "lidar_reliability", "lidar_history", "lidar_dulability", "lidar_depth");
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_profile_lidar, std::bind(&ObjectTracking::scanCallback, this, std::placeholders::_1));

    // tracks with header topic
    rclcpp::QoS qos_profile_tracking = ros2_cpp_utils::utils::getQoS(this, "tracks_reliability", "tracks_history", "tracks_dulability", "tracks_depth");
    object_publisher_ = this->create_publisher<track_msgs::msg::TrackArray>(
        "tracks", qos_profile_tracking
    );

    // visualizer qos
    rclcpp::QoS qos_profile_visualization = ros2_cpp_utils::utils::getQoS(this, "visual_reliability", "visual_history", "visual_dulability", "visual_depth");
    static_obs_posi_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "objects/static/position", qos_profile_visualization);
    static_obs_posi_cov_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "objects/static/position/covariance", qos_profile_visualization);
    static_obs_vel_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "objects/static/velocity", qos_profile_visualization);
    static_obs_traj_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "objects/static/trajectory", qos_profile_visualization);
    dynamic_obs_posi_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "objects/dynamic/position", qos_profile_visualization);
    dynamic_obs_next_posi_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "objects/dynamic/next_position", qos_profile_visualization);
    dynamic_obs_posi_cov_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "objects/dynamic/position/covariance", qos_profile_visualization);
    dynamic_obs_vel_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "objects/dynamic/velocity", qos_profile_visualization);
    dynamic_obs_traj_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "objects/dynamic/trajectory", qos_profile_visualization);
    int freq = ros2_cpp_utils::utils::getRosParam<int>(this, "frequency", 20, 
        "detection callback frequency", 
        "has to be less than lidar scan frequency");
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000/freq),
        std::bind(&ObjectTracking::trackingCallback, this)
    );

    prev_stamp_ = this->get_clock()->now();

    RCLCPP_INFO(get_logger(), "%s start!!\n", this->get_name());
}
ObjectTracking::~ObjectTracking(){}

void ObjectTracking::initializeLaserScanFilter()
{
    // param wrt laser scan filter
    float range_max = ros2_cpp_utils::utils::getRosParam<double>(this, "range_max", 30.0, 
        "max distance from lasescan frame to object to be detected");
    float range_min = ros2_cpp_utils::utils::getRosParam<double>(this, "range_min", 0.06, 
        "min distance from sensor frame to object to be detected");
    float angle_max = ros2_cpp_utils::utils::getRosParam<double>(this, "angle_max", 6.2831852, 
        "max angle in a horizontal direction to object to be detected");
    float angle_min = ros2_cpp_utils::utils::getRosParam<double>(this, "angle_min", 0.0, 
        "min angle in a horizontal direction to object to be detected");
    int ma_interval = ros2_cpp_utils::utils::getRosParam<int>(this, "ma_interval", 100, 
        "ave of ma_interval samples for moving average pre-process", 
        "resolution of laserscan, ma_interval and cluster_points highly depend on each other");
    laser_scan_filter_ = LaserScanFilter(range_max, range_min, angle_max, angle_min, ma_interval);

    // param wrt clustering objects
    cluster_points_ = ros2_cpp_utils::utils::getRosParam<int>(this, "cluster_points", 30, 
        "clustering an object", 
        "resolution of laserscan, ma_interval and cluster_points highly depend on each other");
    object_center_ = cluster_points_ / 2;
    cluster_angle_tolerance_ = ros2_cpp_utils::utils::getRosParam<double>(this, "cluster_angle_tolerance", 0.5, 
        "gap between two objects in a horizontal direction", 
        "highly depending on situation (e.g. large number under the situation that there are many objects)");
    cluster_dist_tolerance_ = ros2_cpp_utils::utils::getRosParam<double>(this, "cluster_dist_tolerance", 0.5, 
        "gap between two objects in a radial direction", 
        "highly depending on situation (e.g. large number under the situation that there are many objects)");

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
}

void ObjectTracking::initializeKFSystem()
{
    // noise param 
    float position_noise_variance = ros2_cpp_utils::utils::getRosParam<double>(this, "position_noise_variance", 1.0, 
        "variance wrt position noise");
    float velocity_noise_variance = ros2_cpp_utils::utils::getRosParam<double>(this, "velocity_noise_variance", 1.0, 
        "variance wrt velocity noise");
    float observation_noise_variance = ros2_cpp_utils::utils::getRosParam<double>(this, "observation_noise_variance", 1.0, 
        "variance wrt observation noise");
    // initial covariance param 
    float position_error_variance = ros2_cpp_utils::utils::getRosParam<double>(this, "position_error_variance", 1.0, 
        "variance wrt position error");
    float velocity_error_variance = ros2_cpp_utils::utils::getRosParam<double>(this, "velocity_error_variance", 1.0, 
        "variance wrt velocity error");
    // noise
    Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity(4, 4);
    process_noise.block(0,0, 2,2) *= position_noise_variance;
    process_noise.block(2,2, 2,2) *= velocity_noise_variance;
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
    system_.reset(new System(process_noise, observation_noise, system_model, control_model, observation_model));

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
    frames_limit_ = ros2_cpp_utils::utils::getRosParam<int>(this, "frames_limit", 1, 
        "frame out if object has not been corrected for over this");
    velocity_limit_ = ros2_cpp_utils::utils::getRosParam<double>(this, "velocity_limit", 1.0, 
        "untracked if object velocity is higher than this");
    matching_dist_ = ros2_cpp_utils::utils::getRosParam<double>(this, "matching_dist", 1.0, 
        "matching with observation within this distance");
    maha_dist_sigma_ = ros2_cpp_utils::utils::getRosParam<double>(this, "maha_dist_sigma", 1.0, 
        "outlier if out of range of this sigma threshe");
    maha_dist_sigma_2_ = std::pow(maha_dist_sigma_, 2.0);
    velocity_sta2dyn_ = ros2_cpp_utils::utils::getRosParam<double>(this, "velocity_sta2dyn", 1.0, 
        "velocity criteria to determine dynamic or static object");
    frames_sta2dyn_ = ros2_cpp_utils::utils::getRosParam<int>(this, "frames_sta2dyn", 20, 
        "the number of frames to be changed from static to dynamic");
    frames_dyn2sta_ = ros2_cpp_utils::utils::getRosParam<int>(this, "frames_dyn2sta", 20, 
        "the number of frames to be changed from dynamic to static");
    time_horizon_ = ros2_cpp_utils::utils::getRosParam<double>(this, "time_horizon", 1.0, 
        "person position in this time step");
    max_track_num_ = ros2_cpp_utils::utils::getRosParam<int>(this, "max_track_num", 1000);

    std::random_device rd;
    gen_= std::mt19937(rd());
    distr_ = std::uniform_int_distribution<>(1, max_track_num_);

    visualize_covariance_ = ros2_cpp_utils::utils::getRosParam<bool>(this, "visualize_covariance", false);
    visualize_trajectory_ = ros2_cpp_utils::utils::getRosParam<bool>(this, "visualize_trajectory", false);
    visualize_next_position_ = ros2_cpp_utils::utils::getRosParam<bool>(this, "visualize_next_position", false);
    
    RCLCPP_INFO(get_logger(), "\tframes_limit: %d", frames_limit_);
    RCLCPP_INFO(get_logger(), "\tvelocity_limit: %.3lf", velocity_limit_);
    RCLCPP_INFO(get_logger(), "\tmatching_dist: %.3lf", matching_dist_);
    RCLCPP_INFO(get_logger(), "\tmaha_dist_sigma: %.3lf", maha_dist_sigma_);
    RCLCPP_INFO(get_logger(), "\tvelocity_sta2dyn: %.3lf", velocity_sta2dyn_);
    RCLCPP_INFO(get_logger(), "\tframes_dyn2sta: %d", frames_dyn2sta_);
    RCLCPP_INFO(get_logger(), "\tframes_sta2dyn: %d", frames_sta2dyn_);
    RCLCPP_INFO(get_logger(), "\tfuture_time: %.3lf", time_horizon_);
    RCLCPP_INFO(get_logger(), "\tmax_track_num: %d", max_track_num_);
    RCLCPP_INFO(get_logger(), "\tvisualize_covariance: %d", visualize_covariance_);
    RCLCPP_INFO(get_logger(), "\tvisualize_trajectory: %d", visualize_trajectory_);
    RCLCPP_INFO(get_logger(), "\tvisualize_next_position: %d", visualize_next_position_);
}

void ObjectTracking::scanCallback(const sensor_msgs::msg::LaserScan &msg){scan_ = msg;}

void ObjectTracking::trackingCallback()
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
    namespace rviz_utils = ros2_cpp_utils::rviz;

    std_msgs::msg::ColorRGBA light_green = rviz_utils::getColor(rviz_utils::GREEN, 1.0);
    std_msgs::msg::ColorRGBA dark_green = rviz_utils::getColor(rviz_utils::BLUE_GREEN, 1.0);
    std_msgs::msg::Header header = rviz_utils::getHeader(stamp, odom_frame_);

    visualization_msgs::msg::MarkerArray markers, markers_vel; // static osb position & velocity marker array
    visualization_msgs::msg::MarkerArray dyn_markers, dyn_markers_vel; // dynamic obs position & velocity marker array
    visualization_msgs::msg::MarkerArray dyn_next_markers; // dynamic obs next position marker array
    visualization_msgs::msg::MarkerArray markers_pos_cov, markers_traj; // static obs position covariance & trajectory marker array
    visualization_msgs::msg::MarkerArray dyn_markers_pos_cov, dyn_markers_traj; // dynamic obs position covariance & trajectory marker array
    
    track_msgs::msg::TrackArray tracks;
    tracks.header = header;

    visualization_msgs::msg::Marker marker = visualization_msgs::msg::MarkerTemplate(header, visualization_msgs::msg::Marker::CYLINDER);
    marker.pose.position.z = 0.2;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.4;

    // duplicate for text visualization
    visualization_msgs::msg::Marker marker_id_text = visualization_msgs::msg::MarkerTemplate(header, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, rviz_utils::BLACK);
    marker_id_text.pose.position.z = 0.5;
    marker_id_text.scale.x = 0.2;
    marker_id_text.scale.y = 0.2;
    marker_id_text.scale.z = 0.4;

    // duplicate velicity marker
    visualization_msgs::msg::Marker marker_vel = visualization_msgs::msg::MarkerTemplate(header, visualization_msgs::msg::Marker::ARROW);
    marker_vel.pose.position.z = 0.2;
    marker_vel.scale.x = 0.2;
    marker_vel.scale.y = 0.2;
    marker_vel.scale.z = 0.4;
    marker_vel.scale.x = 0.0;
    marker_vel.scale.y = 0.1;
    marker_vel.scale.z = 0.1;

    // duplicate for covariance 
    visualization_msgs::msg::Marker marker_pos_cov = visualization_msgs::msg::MarkerTemplate(header, visualization_msgs::msg::Marker::CYLINDER);

    // duplicate for trajectory
    visualization_msgs::msg::Marker marker_traj =visualization_msgs::msg::MarkerTemplate(header, visualization_msgs::msg::Marker::LINE_STRIP);
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