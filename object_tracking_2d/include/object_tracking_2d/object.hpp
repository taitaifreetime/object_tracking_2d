/**
 * @file object.hpp
 * @author tomine (tomine@aisl.cs.tut.ac.jp)
 * @brief object class to manage state 
 * @version 0.1
 * @date 2024-09-25
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef OBJECT_HPP
#define OBJECT_HPP

#include "kalman_filter.hpp"
#include "system.hpp"
#include <memory>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "track_msgs/msg/track.hpp"
#include <rclcpp/rclcpp.hpp>

namespace state_estimation
{


class Object
{
    public: 
        /**
         * @brief Construct a new Object object
         * 
         * @param system to be estimated
         * @param observation 
         * @param initial_covariance 
         * @param id 
         */
        Object(
            std::shared_ptr<System> system, 
            const Eigen::VectorXf &observation, 
            const Eigen::MatrixXf &initial_covariance,
            int id
        );
        ~Object();

        /**
         * @brief prediction step
         * 
         * @param cur_stamp 
         * @param prev_stamp 
         * @param control 
         */
        void predict(
            const rclcpp::Time &cur_stamp, 
            const rclcpp::Time &prev_stamp, 
            const Eigen::VectorXf &control);

        /**
         * @brief correction step
         * 
         * @param observation 
         * @param frames_sta2dyn frames to switch static to dynamic
         * @param frames_dyn2sta frames to switch dynamic to stati 
         * @param vel_thresh 
         */
        void correct(
            const Eigen::VectorXf &observation, 
            int frames_sta2dyn, int frames_dyn2sta,  
            const double &vel_thresh);

        /**
         * @brief add current position with timestamp to own trajectory 
         * 
         * @param stamp 
         */
        void addCurrentTrajectory(const rclcpp::Time &stamp);

        /**
         * @brief squared mahalanobis distance
         * 
         * @param observation 
         * @return double 
         */
        double computeSquaredMahaDistance(const Eigen::VectorXf &observation) const;

        /**
         * @brief euclid distance
         * 
         * @param observation 
         * @return double 
         */
        double computeEuclDistance(const Eigen::VectorXf &observation) const;

        /**
         * @brief tracking info to ros msg
         * 
         * @return track_msgs::msg::Track 
         */
        track_msgs::msg::Track toTrackMsg() const
        {
            track_msgs::msg::Track track;
            track.id = this->id();
            track.label = this->isDynamic() ? "dynamic" : "static";
            track.position.x = this->position()[0];
            track.position.y = this->position()[1];
            track.position.z = 0.2;
            track.velocity.x = this->velocity()[0];
            track.velocity.y = this->velocity()[1];
            track.velocity.z = 0.0;
            Eigen::Matrix4f covariance = this->covariance();
            track.covariance.insert(track.covariance.end(), covariance.data(), covariance.data() + covariance.size());
            track.trajectory = this->trajectory();
            return track;
        }

        // getter
        int id() const {return id_;}
        Eigen::VectorXf state() const {return kf_->getState();}
        Eigen::VectorXf position() const {return kf_->getState().head<2>();}
        Eigen::VectorXf velocity() const {return kf_->getState().segment(2, 2);}
        Eigen::VectorXf acceleration() const {return kf_->getState().tail<2>();}
        Eigen::MatrixXf covariance() const {return kf_->getCovariance().block(0,0, 4,4);}
        std::vector<geometry_msgs::msg::PointStamped> trajectory() const {return this->trajectory_;}

        // thresholding
        bool isUncorrectedOverThan(int frames_limit) const {return frames_limit < frames_uncorrected_;}
        bool isHigherSpeedThan(const double &velocity_limit) const 
        {
            Eigen::Vector2f velocity = this->velocity();
            return velocity_limit < velocity.norm();
        }
        bool isHigherAccelerationThan(const double &acceleration_limit) const
        {
            Eigen::Vector2f acceleration = this->acceleration();
            return acceleration_limit < acceleration.norm();
        }
        bool isFarerThan(const double &dist_limit) const 
        {
            Eigen::Vector2f position = this->position();
            return dist_limit < position.norm();
        }
        bool isStillTentative() const {return tentative_;}
        bool isFramesOld() const {return age_;}
        bool isDynamic() const {return dynamic_;}

        std::shared_ptr<KalmanFilter> kf_;

    private: 
        int id_;
        bool tentative_, dynamic_;
        int frames_dynamic_, frames_static_;
        int frames_uncorrected_;
        int age_;
        Eigen::Matrix2f inversed_posi_cov_;
        std::vector<geometry_msgs::msg::PointStamped> trajectory_;
};


} // namespace state_estimation

#endif // OBJECT_HPP