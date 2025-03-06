#include "linear_feedback_controller/state_compensation_controller.hpp"

#include <sstream>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>

namespace linear_feedback_controller {

StateCompensationController::StateCompensationController() {}

StateCompensationController::~StateCompensationController() {}

void StateCompensationController::initialize(const RobotModelBuilder::SharedPtr& rmb, std::vector<double> gains) {
  
  rmb_ = rmb;
  const auto nq = rmb_->get_nq();
  const auto nv = rmb_->get_nv();
  const auto joint_nq = rmb_->get_joint_nq();
  const auto joint_nv = rmb_->get_joint_nv();
  
  if (gains.size() != joint_nv*2) {
    std::ostringstream oss;
    oss << "StateCompensationController gains have the wrong size. Got " << gains.size() << ". Expects " << joint_nv*2;
    throw std::invalid_argument(oss.str());
  }
  desired_configuration_ = Eigen::VectorXd::Zero(nq);
  desired_velocity_ = Eigen::VectorXd::Zero(nv);
  measured_configuration_ = Eigen::VectorXd::Zero(nq);
  measured_velocity_ = Eigen::VectorXd::Zero(nv);
  control_ = Eigen::VectorXd::Zero(joint_nv);
  diff_state_ = Eigen::VectorXd::Zero(2 * nv);

  gain_ = Eigen::VectorXd::Map(gains.data(), gains.size());
  std::cout << "Gain: " << gain_.transpose() << std::endl;
}

const Eigen::VectorXd& StateCompensationController::compute_control(
    const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg,
    const linear_feedback_controller_msgs::Eigen::Control& control_msg) {
    
    const linear_feedback_controller_msgs::Eigen::JointState& sensor_js =
        sensor_msg.joint_state;
    const linear_feedback_controller_msgs::Eigen::JointState& ctrl_js =
        control_msg.initial_state.joint_state;
    const linear_feedback_controller_msgs::Eigen::Sensor& ctrl_init =
        control_msg.initial_state;
    pinocchio::Model const& model = rmb_->get_model();
    pinocchio::Data& data = rmb_->get_data();

    // Reconstruct the state vector: x = [q, v]
    rmb_->construct_robot_state(ctrl_init, desired_configuration_,
                                desired_velocity_);
    rmb_->construct_robot_state(sensor_msg, measured_configuration_,
                                measured_velocity_);

  // Step 1: calculate the desired state by integrating the control for the correct amount of time.
  double time = (sensor_msg.joint_state.stamp - control_msg.initial_state.joint_state.stamp).seconds();
  integrate(control_msg.feedforward, time);

  // Step 2: calculate the deviation from desired state and translate it into acceleration correction
  pinocchio::difference(model, measured_configuration_,
                        desired_configuration_,
                        diff_state_.head(model.nv));
  diff_state_.tail(model.nv) = desired_velocity_ - measured_velocity_;

  // Step 3: calculate the corrected torque based on the acceleration correction.
  pinocchio::crba(model, data, measured_configuration_);
  control_.noalias() = control_msg.feedforward;
  control_.noalias() += data.M * gain_.cwiseProduct(diff_state_);

  return control_;
}

void StateCompensationController::integrate(Eigen::VectorXd const& u, double time) {
  // TODO acceleration_ can be called only once.
  acceleration_ = pinocchio::aba(rmb_->get_model(), rmb_->get_data(), desired_configuration_, desired_velocity_, u);

  desired_velocity_ += acceleration_ * time;
  dq_ = desired_velocity_ * time;

  pinocchio::integrate(rmb_->get_model(), desired_configuration_, dq_, desired_configuration_);
}

}  // namespace linear_feedback_controller
