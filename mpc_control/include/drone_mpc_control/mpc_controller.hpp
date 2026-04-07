#ifndef DRONE_MPC_CONTROL_MPC_CONTROLLER_HPP
#define DRONE_MPC_CONTROL_MPC_CONTROLLER_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <chrono>

namespace drone_mpc_control {

class MPCController {
public:
  MPCController();
  ~MPCController();

  struct State {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
  };

  struct ControlInput {
    Eigen::Vector4d thrust;
  };

  struct TrajectoryPoint {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    double time;
  };

  void configure(double horizon, double dt, double mass,
                 const Eigen::Vector3d& gravity = Eigen::Vector3d(0, 0, -9.81));

  void setReferenceTrajectory(const std::vector<TrajectoryPoint>& trajectory);

  ControlInput computeControl(const State& current_state, const TrajectoryPoint& desired_state);

  void reset();

  int getHorizon() const { return horizon_; }

  struct SolverStats {
    double solve_time_ms;
    int iterations;
    bool success;
  };
  SolverStats getLastSolveStats() const { return last_stats_; }

private:
  int horizon_;
  double dt_;
  double mass_;
  Eigen::Vector3d gravity_;

  double position_weight_;
  double velocity_weight_;
  double acceleration_weight_;
  double control_weight_;

  double min_thrust_;
  double max_thrust_;

  std::vector<TrajectoryPoint> reference_trajectory_;

  State state_;

  SolverStats last_stats_;

  Eigen::Matrix<double, 12, 12> getStateMatrix(double dt) const;
  Eigen::Matrix<double, 12, 4> getInputMatrix(double dt) const;

  void predictStates(const Eigen::Matrix<double, 12, 1>& current_state,
                     const std::vector<Eigen::Vector4d>& control_sequence,
                     std::vector<Eigen::Matrix<double, 12, 1>>& predicted_states);

  double computeCost(const std::vector<Eigen::Vector4d>& control_sequence,
                     const std::vector<Eigen::Matrix<double, 12, 1>>& predicted_states,
                     const std::vector<TrajectoryPoint>& reference);

  bool checkConstraints(const std::vector<Eigen::Vector4d>& control_sequence);

  std::vector<Eigen::Vector4d> optimizeControl(const State& current_state,
                                                const TrajectoryPoint& desired_state);
};

} 

#endif 
