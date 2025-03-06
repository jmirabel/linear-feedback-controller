#ifndef LINEAR_FEEDBACK_CONTROLLER_STATECOMPENSATIONCONTROLLER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_STATECOMPENSATIONCONTROLLER_HPP

#include "Eigen/Core"
#include "linear_feedback_controller/robot_model_builder.hpp"
#include "linear_feedback_controller_msgs/eigen_conversions.hpp"

namespace linear_feedback_controller {

class StateCompensationController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static constexpr int kNbFreeFlyerDof = 6;

  StateCompensationController();
  virtual ~StateCompensationController();

  void initialize(const RobotModelBuilder::SharedPtr& rmb, std::vector<double> gains);

  const Eigen::VectorXd& compute_control(
      const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg,
      const linear_feedback_controller_msgs::Eigen::Control& control_msg);

 private:
  Eigen::VectorXd desired_configuration_;
  Eigen::VectorXd desired_velocity_;
  Eigen::VectorXd measured_configuration_;
  Eigen::VectorXd measured_velocity_;

  void integrate(Eigen::VectorXd const& u, double time);

  /**
   *  @brief Difference between the measured and the desired state,
   *  \$f x = [ q^T, \dot{q}^T ]^T \$f
   *
   */
  Eigen::VectorXd diff_state_;
  Eigen::VectorXd acceleration_;
  Eigen::VectorXd dq_;

  Eigen::VectorXd gain_;
  Eigen::VectorXd control_;
  RobotModelBuilder::SharedPtr rmb_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_STATECOMPENSATIONCONTROLLER_HPP
