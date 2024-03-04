//
// Created by pol on 14.06.22.
//

#ifndef DYNAMIC_MAPPING_CPP_GAUSSIANSTATE_H
#define DYNAMIC_MAPPING_CPP_GAUSSIANSTATE_H

#include <Eigen/Core>

template <size_t Dim>
struct GaussianState {
  Eigen::Matrix<double, Dim, 1> state;
  Eigen::Matrix<double, Dim, Dim> covar;
  GaussianState(Eigen::Matrix<double, Dim, 1> state, Eigen::Matrix<double, Dim, Dim> covar) : state(state), covar(covar) {}
  GaussianState() : state(Dim), covar(Dim, Dim) {}
};

#endif  // DYNAMIC_MAPPING_CPP_GAUSSIANSTATE_H
