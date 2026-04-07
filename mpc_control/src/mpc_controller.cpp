#include "drone_mpc_control/mpc_controller.hpp"

namespace drone_mpc_control {

MPCController::MPCController()
  : horizon_(20),
    dt_(0.05),
    mass_(1.5),
    gravity_(0, 0, -9.81),
    position_weight_(10.0),
    velocity_weight_(5.0),
    acceleration_weight_(1.0),
    control_weight_(0.1),
    min_thrust_(0.0),
    max_thrust_(15.0) {
}

MPCController::~MPCController() = default;

void MPCController::configure(double horizon, double dt, double mass,
                               const Eigen::Vector3d& gravity) {
  horizon_ = static_cast<int>(horizon);
  dt_ = dt;
  mass_ = mass;
  gravity_ = gravity;
}

void MPCController::setReferenceTrajectory(const std::vector<TrajectoryPoint>& trajectory) {
  reference_trajectory_ = trajectory;
}

MPCController::ControlInput MPCController::computeControl(
    const State& current_state,
    const TrajectoryPoint& desired_state) {

  state_ = current_state;
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  std::vector<Eigen::Vector4d> control_sequence = optimizeControl(current_state, desired_state);
  
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  
  last_stats_.solve_time_ms = duration.count() / 1000.0;
  last_stats_.success = !control_sequence.empty();
  last_stats_.iterations = 100;
  
  ControlInput control;
  if (control_sequence.empty()) {
    control.thrust = Eigen::Vector4d::Zero();
  } else {
    control.thrust = control_sequence[0];
  }
  
  return control;
}

void MPCController::reset() {
  state_ = State{};
  reference_trajectory_.clear();
  last_stats_ = SolverStats{};
}

Eigen::Matrix<double, 12, 12> MPCController::getStateMatrix(double dt) const {
  Eigen::Matrix<double, 12, 12> A = Eigen::Matrix<double, 12, 12>::Identity();
  
  A.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
  
  return A;
}

Eigen::Matrix<double, 12, 4> MPCController::getInputMatrix(double dt) const {
  Eigen::Matrix<double, 12, 4> B = Eigen::Matrix<double, 12, 4>::Zero();
  
  B.block<3, 4>(6, 0) = Eigen::Matrix<double, 3, 4>::Ones() * dt / mass_;
  
  return B;
}

void MPCController::predictStates(
    const Eigen::Matrix<double, 12, 1>& current_state,
    const std::vector<Eigen::Vector4d>& control_sequence,
    std::vector<Eigen::Matrix<double, 12, 1>>& predicted_states) {

  predicted_states.clear();
  predicted_states.resize(horizon_);
  
  Eigen::Matrix<double, 12, 1> state = current_state;
  
  for (int i = 0; i < horizon_; ++i) {
    Eigen::Matrix<double, 12, 12> A = getStateMatrix(dt_);
    Eigen::Matrix<double, 12, 4> B = getInputMatrix(dt_);
    Eigen::Matrix<double, 12, 1> gravity_input;
    gravity_input.segment<3>(6) = gravity_;
    
    state = A * state + B * control_sequence[i] + gravity_input;
    predicted_states[i] = state;
  }
}

double MPCController::computeCost(
    const std::vector<Eigen::Vector4d>& control_sequence,
    const std::vector<Eigen::Matrix<double, 12, 1>>& predicted_states,
    const std::vector<TrajectoryPoint>& reference) {

  double cost = 0.0;
  
  for (int i = 0; i < horizon_; ++i) {
    Eigen::Vector3d pos_error = predicted_states[i].segment<3>(0) - reference[i].position;
    Eigen::Vector3d vel_error = predicted_states[i].segment<3>(3) - reference[i].velocity;
    Eigen::Vector3d acc_error = predicted_states[i].segment<3>(6) - reference[i].acceleration;
    
    cost += position_weight_ * pos_error.squaredNorm();
    cost += velocity_weight_ * vel_error.squaredNorm();
    cost += acceleration_weight_ * acc_error.squaredNorm();
    cost += control_weight_ * control_sequence[i].squaredNorm();
  }
  
  return cost;
}

bool MPCController::checkConstraints(const std::vector<Eigen::Vector4d>& control_sequence) {
  for (const auto& control : control_sequence) {
    for (int i = 0; i < 4; ++i) {
      if (control[i] < min_thrust_ || control[i] > max_thrust_) {
        return false;
      }
    }
  }
  return true;
}

std::vector<Eigen::Vector4d> MPCController::optimizeControl(
    const State& current_state,
    const TrajectoryPoint& desired_state) {

  std::vector<Eigen::Vector4d> control_sequence(horizon_);
  
  for (auto& control : control_sequence) {
    control = Eigen::Vector4d::Constant(mass_ * 9.81 / 4.0);
  }
  
  std::vector<TrajectoryPoint> reference(horizon_);
  for (int i = 0; i < horizon_; ++i) {
    reference[i].position = desired_state.position;
    reference[i].velocity = desired_state.velocity;
    reference[i].acceleration = desired_state.acceleration;
  }
  
  Eigen::Matrix<double, 12, 1> state_vec;
  state_vec.segment<3>(0) = current_state.position;
  state_vec.segment<3>(3) = current_state.velocity;
  state_vec.segment<3>(6) = Eigen::Vector3d::Zero();
  state_vec.segment<3>(9) = current_state.angular_velocity;
  
  const int max_iterations = 100;
  const double learning_rate = 0.01;
  
  for (int iter = 0; iter < max_iterations; ++iter) {
    std::vector<Eigen::Matrix<double, 12, 1>> predicted_states;
    predictStates(state_vec, control_sequence, predicted_states);
    
    std::vector<Eigen::Vector4d> gradients(horizon_);
    for (int i = 0; i < horizon_; ++i) {
      gradients[i] = Eigen::Vector4d::Zero();
    }
    
    for (int i = 0; i < horizon_; ++i) {
      Eigen::Vector3d pos_error = predicted_states[i].segment<3>(0) - reference[i].position;
      Eigen::Vector3d vel_error = predicted_states[i].segment<3>(3) - reference[i].velocity;
      Eigen::Vector3d acc_error = predicted_states[i].segment<3>(6) - reference[i].acceleration;
      
      Eigen::Vector3d gradient_state = position_weight_ * pos_error +
                                       velocity_weight_ * vel_error +
                                       acceleration_weight_ * acc_error;
      
      for (int j = i; j < horizon_; ++j) {
        Eigen::Matrix<double, 12, 4> B = getInputMatrix(dt_);
        gradients[j] += B.transpose() * gradient_state;
      }
      
      gradients[i] += control_weight_ * control_sequence[i];
    }
    
    for (int i = 0; i < horizon_; ++i) {
      control_sequence[i] -= learning_rate * gradients[i];
      
      control_sequence[i] = control_sequence[i].cwiseMax(min_thrust_).cwiseMin(max_thrust_);
    }
    
    std::vector<Eigen::Matrix<double, 12, 1>> final_predicted_states;
    predictStates(state_vec, control_sequence, final_predicted_states);
    
    double cost = computeCost(control_sequence, final_predicted_states, reference);
    if (cost < 0.001) {
      break;
    }
  }
  
  return control_sequence;
}

} 
