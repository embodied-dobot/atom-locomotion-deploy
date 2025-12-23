// Copyright (c) 2025 Shenzhen Yuejiang Technology Co., Ltd.
// SPDX-License-Identifier: Apache-2.0

// Common math/types helpers and simple RL model interfaces.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include "common/robot_state.h"

using scalar_t = float;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;
using tensor_element_t = float;

struct Control {
  float x = 0.0f;
  float y = 0.0f;
  float yaw = 0.0f;
};

struct ControlFiltered {
  float x = 0.0f;
  float y = 0.0f;
  float yaw = 0.0f;
};

struct Observations {
  vector3_t ang_vel;
  vector3_t gravity_vec;
  quaternion_t base_quat;

  vector_t dof_pos;
  vector_t dof_vel;
  vector_t actions;
};

struct YamlParams {
  YAML::Node config_node;

  template <typename T>
  T Get(const std::string& key, const T& default_value = T()) const {
    if (config_node[key]) {
      return config_node[key].as<T>();
    }
    return default_value;
  }

  bool Has(const std::string& key) const {
    return config_node[key].IsDefined();
  }
};

template <>
inline vector_t YamlParams::Get<vector_t>(const std::string& key,
                                          const vector_t& default_value) const {
  if (!config_node[key]) {
    return default_value;
  }

  std::vector<scalar_t> vec = config_node[key].as<std::vector<scalar_t>>();
  return Eigen::Map<const vector_t>(vec.data(), vec.size());
}

inline scalar_t lowpass(scalar_t current, scalar_t previous, scalar_t alpha) {
  return alpha * current + (1.0 - alpha) * previous;
}

inline vector3_t QuatRotateInverse(const quaternion_t& q, const vector3_t& v) {
  scalar_t q_w = q.w();
  vector3_t q_vec(q.x(), q.y(), q.z());

  vector3_t result = v * (2.0 * q_w * q_w - 1.0);
  result -= 2.0 * q_w * q_vec.cross(v);
  result += 2.0 * q_vec * q_vec.dot(v);

  return result;
}

inline vector3_t quatToXyz(const quaternion_t& q) {
  vector3_t xyz;

  // Roll (X-axis rotation)
  scalar_t sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  scalar_t cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  xyz(0) = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (Y-axis rotation)
  scalar_t sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1) {
    xyz(1) = std::copysign(M_PI / 2, sinp);
  } else {
    xyz(1) = std::asin(sinp);
  }

  // Yaw (Z-axis rotation)
  scalar_t siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  scalar_t cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  xyz(2) = std::atan2(siny_cosp, cosy_cosp);

  return xyz;
}

struct OnnxTensor {
  std::shared_ptr<Ort::Env> onnxEnvPrt;
  std::unique_ptr<Ort::Session> sessionPtr;
  std::vector<const char*> inputNames;
  std::vector<const char*> outputNames;
  std::vector<Ort::AllocatedStringPtr> inputNodeNameAllocatedStrings;
  std::vector<Ort::AllocatedStringPtr> outputNodeNameAllocatedStrings;
  std::vector<std::vector<int64_t>> inputShapes;
  std::vector<std::vector<int64_t>> outputShapes;
};

class RobotModel {
 public:
  explicit RobotModel(const YamlParams& config_params)
      : params(std::make_shared<YamlParams>(config_params)) {}

  virtual ~RobotModel() = default;

  virtual vector_t compute_observation(const Atom::BaseState& base_state,
                                       const Atom::JointState& leg_state,
                                       const Atom::ArmJointState& arm_state,
                                       const Control& control,
                                       Observations& obs) = 0;

  virtual vector_t forward(const vector_t& observation) = 0;

  virtual std::string name() const = 0;

 protected:
  std::shared_ptr<YamlParams> params;
};
