#include <object_tracking_2d/object.hpp>

namespace state_estimation
{

Object::Object(
    std::shared_ptr<System> system,
    const Eigen::VectorXf &observation, 
    const Eigen::MatrixXf &initial_covariance, 
    int id
){
    id_ = id;
    tentative_ = true;
    dynamic_ = false;
    frames_dynamic_ = 0;
    frames_static_ = 0;
    frames_uncorrected_ = 0;
    age_ = 0;
    kf_.reset(new KalmanFilter(system, observation, initial_covariance));
}

Object::~Object(){}

void Object::predict(
    const rclcpp::Time &cur_stamp,
    const rclcpp::Time &prev_stamp, 
    const Eigen::VectorXf &control)
{
    this->kf_->predict((cur_stamp-prev_stamp).nanoseconds()/1000000000.0, control); // must
    this->addCurrentTrajectory(cur_stamp);
    this->frames_uncorrected_++;
    this->age_++;
    this->inversed_posi_cov_ = this->covariance().block(0,0, 2,2).inverse();
}

void Object::correct(
    const Eigen::VectorXf &observation, 
    int frames_sta2dyn, int frames_dyn2sta,  
    const double &vel_thresh)
{
    this->kf_->correct(observation); // must
    this->frames_uncorrected_ = 0;
    this->tentative_ = false;
    if (this->frames_dynamic_ > frames_sta2dyn) this->dynamic_ = true;
    else if (this->frames_static_ > frames_dyn2sta) this->dynamic_ = false;

    if (this->velocity().norm() > vel_thresh) 
    {
        this->frames_dynamic_++;
        this->frames_static_ = 0;
    }
    else
    {
        this->frames_static_++;
        this->frames_dynamic_ = 0;
    }
}

double Object::computeSquaredMahaDistance(const Eigen::VectorXf &observation) const
{
    Eigen::Vector2f standard_division = this->position() - observation.head<2>();
    double maha_distance = standard_division.transpose()*inversed_posi_cov_*standard_division;
    return maha_distance;
}
double Object::computeEuclDistance(const Eigen::VectorXf &observation) const
{
    Eigen::Vector2f observed_position = observation.head<2>();
    Eigen::Vector2f estimated_position = this->position();
    double eucl_distance =std::sqrt(
        std::pow(observed_position[0] - estimated_position[0], 2.0) 
        + std::pow(observed_position[1] - estimated_position[1], 2.0));
    return eucl_distance;
}  
void Object::addCurrentTrajectory(const rclcpp::Time &stamp) 
{
    geometry_msgs::msg::PointStamped p;
    p.header.stamp = stamp;
    p.point.x = this->position()[0];
    p.point.y = this->position()[1];
    p.point.z = 0.2;
    if (trajectory_.size() > 20) trajectory_.erase(trajectory_.begin());
    trajectory_.push_back(p);
}

}