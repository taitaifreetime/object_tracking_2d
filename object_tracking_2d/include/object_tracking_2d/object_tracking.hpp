/**
 * @file object_tracking.hpp
 * @author tomine (tomine@aisl.cs.tut.ac.jp)
 * @brief object tracking class using 2d laser scan
 * @version 0.1.0
 * @date 2024-07-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef OBJECT_TRACKING_HPP
#define OBJECT_TRACKING_HPP

#include "object_tracking_2d/laser_scan_filter.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "track_msgs/msg/track_array.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/color_rgba.hpp>
#include "system.hpp"
#include "object_tracking_2d/object.hpp"
#include "Hungarian.h"
#include <random>

namespace state_estimation
{


class ObjectTracking : public rclcpp::Node
{
    public:
        ObjectTracking();
        ~ObjectTracking();

    private:
        void initializeKFSystem();
        void initializeLaserScanFilter();

        void scanCallback(const sensor_msgs::msg::LaserScan &msg) ;

        void trackingCallback();

        /**
         * @brief detect objects
         * 
         * @param original_scans subscribed laserscan msg
         * @param preproc_scans pre-processed laserscan
         */
        std::vector<LaserScanFilter::PreProcScan> computeLocalMinimums(
            const sensor_msgs::msg::LaserScan &original_scans, 
            const std::vector<LaserScanFilter::PreProcScan> &preproc_scans
        ) const;

        void visualizeObjects(const rclcpp::Time &stamp) const;

        // laserscan pre-processor
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_; 
        LaserScanFilter laser_scan_filter_;
        sensor_msgs::msg::LaserScan scan_;
        std::string lidar_frame_, odom_frame_;
        int cluster_points_; // clustering an object
        float cluster_angle_tolerance_; // gap between two objects in a horizontal direction
        float cluster_dist_tolerance_; // gap between two objects in a radial direction
        int object_center_; // cluster_points/2
        
        // callback
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<track_msgs::msg::TrackArray>::SharedPtr object_publisher_;
        track_msgs::msg::TrackArray objects_msg_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_obs_posi_marker_publisher_; 
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_obs_posi_cov_marker_publisher_; 
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_obs_traj_marker_publisher_; 
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_obs_vel_marker_publisher_; 
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obs_posi_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obs_posi_cov_marker_publisher_; 
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obs_traj_marker_publisher_; 
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obs_vel_marker_publisher_; 

        // transformation        
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // system to be estimated for kalman filter
        std::shared_ptr<System> system_;
        Eigen::MatrixXf initial_covariance_;
        std::set<int> existing_ids_;
        std::mt19937 gen_;
        std::uniform_int_distribution<> distr_;
        std::vector<std::shared_ptr<Object>> objects_;

        // threshold for kalman filter tracking
        int frames_limit_;
        double velocity_limit_;
        int max_track_num_;
        double matching_dist_;
        double velocity_sta2dyn_;
        int frames_dyn2sta_, frames_sta2dyn_;
        // visualizer
        bool visualize_;

        rclcpp::Time prev_stamp_;
};


} // namespace state_estimation

#endif // OBJECT_TRACKING_HPP