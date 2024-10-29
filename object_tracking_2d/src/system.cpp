#include "system.hpp"

namespace state_estimation{
    
System::System(
    const Eigen::MatrixXf &process_noise, 
    const Eigen::MatrixXf &observation_noise, 
    const Eigen::MatrixXf &system_model, 
    const Eigen::MatrixXf &control_model, 
    const Eigen::MatrixXf &observation_model 
) : process_noise_(process_noise), 
    observation_noise_(observation_noise), 
    A_(system_model), 
    B_(control_model), 
    C_(observation_model)
{}

System::~System(){}

Eigen::VectorXf System::predictState(const double &dt, const Eigen::VectorXf &state, const Eigen::VectorXf &control) 
{
    A_.block(0, 2, 2, 2) = Eigen::Matrix2f::Identity() * dt;
    return A_*state + B_*control;
}

Eigen::VectorXf System::predictObservation(const Eigen::VectorXf &state) 
{
    return C_*state;
}

}