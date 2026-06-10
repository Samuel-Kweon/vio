#include <vio/imu_integrator.hpp>


ImuIntegrator::ImuIntegrator(const ImuState& initial_state, const Eigen::Vector3d& gravity_w){
    gravity_w_ = gravity_w;
    state_ = initial_state; 
}

void ImuIntegrator::integrate(const ImuMeasurement& m){

    if(first_measurement_){
        state_.timestamp_ns = m.timestamp_ns;
        first_measurement_ = false;
        return;
    }

    double dt = (m.timestamp_ns - state_.timestamp_ns) / 1e9;
    Eigen::Vector3d acc_corrected = m.accel - state_.bias_accel;                   //B
    Eigen::Vector3d gyro_corrected = m.gyro - state_.bias_gyro;                    //B
    Eigen::Vector3d acc_world = state_.orientation * acc_corrected + gravity_w_;   //W

    Eigen::Vector3d p_new = state_.position + state_.velocity * dt;                //W
    Eigen::Vector3d v_new = state_.velocity + acc_world * dt;                      //W
    
    //Eigen::Queaterniond// orientation update
    double angle = gyro_corrected.norm() * dt;                                     //B
    Eigen::Vector3d axis;       
    if (angle > 1e-10){
        axis = gyro_corrected.normalized();                                        //B
    } else {
        axis = Eigen::Vector3d::UnitX();                                           //B
    }
    Eigen::Quaterniond q_delta(Eigen::AngleAxisd(angle, axis));                     
    state_.orientation = state_.orientation * q_delta;  
    state_.orientation.normalize();                                                //W     

    state_.position = p_new;                                                       //W
    state_.velocity = v_new;                                                       //W
    state_.timestamp_ns = m.timestamp_ns;                                          //W     

}