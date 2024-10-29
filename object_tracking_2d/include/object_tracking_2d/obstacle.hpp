/**
 * @file obstacle.hpp
 * @author tomine (tomine@aisl.cs.tut.ac.jp)
 * @brief obstacle class to manage state 
 * @version 0.1
 * @date 2024-09-25
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include "kalman_filter.hpp"
#include "system.hpp"
#include <memory>
#include <geometry_msgs/msg/point.hpp>

namespace state_estimation
{


class Obstacle
{
    public: 
        Obstacle(
            const double &time,  
            std::shared_ptr<System> system, 
            const Eigen::VectorXf &observation, 
            const Eigen::MatrixXf &initial_covariance,
            int id
        );
        ~Obstacle();

        void predict(const double &dt, const Eigen::VectorXf &control);
        void correct(
            const Eigen::VectorXf &observation, 
            const double &cur_time, 
            int frames_sta2dyn, int frames_dyn2sta,  
            const double &vel_thresh);

        int id() const {return id_;}
        Eigen::VectorXf state() const;
        Eigen::VectorXf position() const;
        Eigen::VectorXf velocity() const;
        Eigen::VectorXf acceleration() const;
        Eigen::MatrixXf covariance() const;
        std::vector<geometry_msgs::msg::Point> trajectory() const {return this->trajectory_;}

        bool isTimeoutOverThan(const double &timeout, const double &cur_time) const {return timeout < cur_time - prev_time_;}
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
        void addCurrentTrajectory();
        double computeSquaredMahaDistance(const Eigen::VectorXf &observation) const;
        double computeEuclDistance(const Eigen::VectorXf &observation) const;

        std::shared_ptr<KalmanFilter> kf_;

    private: 
        int id_;
        bool tentative_, dynamic_;
        int frames_dynamic_, frames_static_;
        int frames_uncorrected_, frames_corrected_;
        int age_;
        double prev_time_;
        Eigen::Matrix2f inversed_posi_cov_;
        std::vector<geometry_msgs::msg::Point> trajectory_;
};


} // namespace state_estimation

#endif // OBSTACLE_HPP