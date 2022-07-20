//
// Created by hannes on 2017-11-29.
//

#ifndef KONTIKIV2_LIDAR_ORIENTATION_MEASUREMENT_H
#define KONTIKIV2_LIDAR_ORIENTATION_MEASUREMENT_H

#include <Eigen/Dense>

#include <iostream>
#include <kontiki/trajectories/trajectory.h>
#include <kontiki/trajectory_estimator.h>
#include "../sensors/lidar.h"

namespace kontiki {
namespace measurements {

template<typename LiDARModel>
class LiDAROrientationMeasurement {
  using Quat = Eigen::Quaternion<double>;
//  Eigen::Quaternion<T>
 public:
  LiDAROrientationMeasurement(std::shared_ptr<LiDARModel> lidar, double t, const Quat &q)
    : lidar_(lidar), t(t), q_(q) {}

  template<typename TrajectoryModel, typename T>
  Eigen::Quaternion<T> Measure(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                 const type::LiDAR<LiDARModel, T> &lidar) const {
    Eigen::Quaternion<T> q_M_I = trajectory.Orientation(T(t));
    const Eigen::Quaternion<T> q_L_I = lidar.relative_orientation();
    Eigen::Quaternion<T> q_M_L = q_M_I * q_L_I.conjugate();
    return q_M_L;
  }

  template<typename TrajectoryModel, typename T>
  T Error(const type::Trajectory<TrajectoryModel, T> &trajectory, const type::LiDAR<LiDARModel, T> &lidar) const {
    Eigen::Quaternion<T> qhat = Measure<TrajectoryModel, T>(trajectory, lidar);
    return q_.cast<T>().angularDistance(qhat);
  }

  // Measurement data
  std::shared_ptr<LiDARModel> lidar_;
  double t;
  Quat q_;

 protected:

  // Residual struct for ceres-solver
  template<typename TrajectoryModel>
  struct Residual {
    Residual(const LiDAROrientationMeasurement<LiDARModel> &m) : measurement(m) {}

    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      size_t offset = 0;
      auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);

      offset += trajectory_meta.NumParameters();
      auto lidar = entity::Map<LiDARModel, T>(&params[offset], lidar_meta);

      residual[0] = measurement.Error<TrajectoryModel, T>(trajectory, lidar);
      return true;
    }

    const LiDAROrientationMeasurement& measurement;
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
    cost_function->SetNumResiduals(1);
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


#endif //KONTIKIV2_LIDAR_ORIENTATION_MEASUREMENT_H
