//
// Created by hannes on 2017-11-29.
//

#ifndef KONTIKIV2_LIDAR_POSITION_MEASUREMENT_H
#define KONTIKIV2_LIDAR_POSITION_MEASUREMENT_H

#include <Eigen/Dense>

#include <iostream>
#include <kontiki/trajectories/trajectory.h>
#include <kontiki/trajectory_estimator.h>
#include "../sensors/lidar.h"

namespace kontiki {
namespace measurements {

template<typename LiDARModel>
class LiDARPositionMeasurement {
  using Vector3 = Eigen::Matrix<double, 3, 1>;
 public:
  LiDARPositionMeasurement(std::shared_ptr<LiDARModel> lidar, double t, const Vector3 &p)
    : lidar_(lidar), t(t), p_(p) {}

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Measure(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                 const type::LiDAR<LiDARModel, T> &lidar) const {
    int flags = trajectories::EvaluationFlags::EvalPosition | trajectories::EvaluationFlags::EvalOrientation;
    auto T_M_I = trajectory.Evaluate(T(t), flags);

    const Eigen::Matrix<T, 3, 1> p_L_I = lidar.relative_position();
    const Eigen::Quaternion<T> q_L_I = lidar.relative_orientation();
    Eigen::Matrix<T, 3, 1> p_I_L = q_L_I.conjugate() * (-p_L_I);

    Eigen::Matrix<T, 3, 1> p_M_L = T_M_I->orientation * p_I_L + T_M_I->position;

    return p_M_L;
  }

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Error(const type::Trajectory<TrajectoryModel, T> &trajectory,
                               const type::LiDAR<LiDARModel, T> &lidar) const {
    Eigen::Matrix<T, 3, 1> p_M_L = p_.cast<T>();
    return p_M_L - Measure<TrajectoryModel, T>(trajectory, lidar);
  }

  template<typename TrajectoryModel>
  Eigen::Matrix<double, 3, 1> Error(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
    return p_ - Measure<TrajectoryModel, double>(trajectory, *lidar_);
  }

  // Measurement data
  std::shared_ptr<LiDARModel> lidar_;
  double t;
  Vector3 p_;

 protected:

  // Residual struct for ceres-solver
  template<typename TrajectoryModel>
  struct Residual {
    Residual(const LiDARPositionMeasurement<LiDARModel> &m) : measurement(m) {}

    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      size_t offset = 0;
      auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);

      offset += trajectory_meta.NumParameters();
      auto lidar = entity::Map<LiDARModel, T>(&params[offset], lidar_meta);

      Eigen::Map<Eigen::Matrix<T,3,1>> r(residual);
      r = measurement.Error<TrajectoryModel, T>(trajectory, lidar);
      return true;
    }

    const LiDARPositionMeasurement& measurement;
    typename TrajectoryModel::Meta trajectory_meta;
    typename LiDARModel::Meta lidar_meta;
  }; // Residual;

  template<typename TrajectoryModel>
  void AddToEstimator(kontiki::TrajectoryEstimator<TrajectoryModel>& estimator) {
    using ResidualImpl = Residual<TrajectoryModel>;
    auto residual = new ResidualImpl(*this);
    auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
    std::vector<entity::ParameterInfo<double>> parameter_info;

    // Add trajectory to problem
    //estimator.trajectory()->AddToProblem(estimator.problem(), residual->meta, parameter_blocks, parameter_sizes);
    estimator.AddTrajectoryForTimes({{t,t}}, residual->trajectory_meta, parameter_info);
    lidar_->AddToProblem(estimator.problem(), {{t,t}}, residual->lidar_meta, parameter_info);


    for (auto& pi : parameter_info) {
      cost_function->AddParameterBlock(pi.size);
    }

    // Add measurement
    cost_function->SetNumResiduals(3);
    // If we had any measurement parameters to set, this would be the place

    // Give residual block to estimator problem
    estimator.problem().AddResidualBlock(cost_function,
                                         nullptr,
                                         entity::ParameterInfo<double>::ToParameterBlocks(parameter_info));
  }

  // TrajectoryEstimator must be a friend to access protected members
  template<template<typename> typename TrajectoryModel>
  friend class kontiki::TrajectoryEstimator;
};


} // namespace measurements
} // namespace kontiki


#endif //KONTIKIV2_LIDAR_POSITION_MEASUREMENT_H
