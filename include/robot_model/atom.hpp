// Copyright (c) 2025 Shenzhen Yuejiang Technology Co., Ltd.
// SPDX-License-Identifier: Apache-2.0

// ONNX-based RL policy model for Atom robot.
#pragma once

#include "utils/utils.hpp"

class ATOMModel : public RobotModel {
 private:
  std::vector<scalar_t> clock_inputs = {0.0f, 0.0f};
  vector_t obs_history_buffer;
  OnnxTensor onnx_tensor;
  int64_t num_output_elements = 1;
  scalar_t gait_phase_ = 0.0f;

  scalar_t updatePhase(const scalar_t cycle_time) {
    gait_phase_ = std::fmod(
        gait_phase_ + params->Get<scalar_t>("inference_dt") / cycle_time, 1.0);

    if (gait_phase_ < 0.0) {
      gait_phase_ += 1.0;
    }
    return gait_phase_;
  }

 public:
  explicit ATOMModel(const YamlParams& config_params)
      : RobotModel(config_params) {
    this->obs_history_buffer =
        vector_t::Zero(params->Get<int>("num_history") *
                       params->Get<int>("num_one_step_observations"));

    this->onnx_tensor.onnxEnvPrt.reset(
        new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "OnnxController"));
    std::string model_path = std::string(POLICY_DIR) + "/model/" +
                             params->Get<std::string>("model_name");
    std::cout << "[Loading HomieModel] model_path: " << model_path
              << std::endl;

    // Create session.
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetInterOpNumThreads(1);
    this->onnx_tensor.sessionPtr = std::make_unique<Ort::Session>(
        *this->onnx_tensor.onnxEnvPrt, model_path.c_str(), sessionOptions);

    // Get input and output info.
    this->onnx_tensor.inputNames.clear();
    this->onnx_tensor.outputNames.clear();
    this->onnx_tensor.inputShapes.clear();
    this->onnx_tensor.outputShapes.clear();

    Ort::AllocatorWithDefaultOptions allocator;
    std::cout << "count: "
              << this->onnx_tensor.sessionPtr->GetInputCount() << std::endl;

    for (int i = 0; i < this->onnx_tensor.sessionPtr->GetInputCount(); i++) {
      auto inputnamePtr =
          this->onnx_tensor.sessionPtr->GetInputNameAllocated(i, allocator);
      this->onnx_tensor.inputNodeNameAllocatedStrings.push_back(
          std::move(inputnamePtr));
      this->onnx_tensor.inputNames.push_back(
          this->onnx_tensor.inputNodeNameAllocatedStrings.back().get());
      this->onnx_tensor.inputShapes.push_back(
          this->onnx_tensor.sessionPtr->GetInputTypeInfo(i)
              .GetTensorTypeAndShapeInfo()
              .GetShape());
      std::vector<int64_t> shape =
          this->onnx_tensor.sessionPtr->GetInputTypeInfo(i)
              .GetTensorTypeAndShapeInfo()
              .GetShape();
      std::cerr << "Shape: [";
      for (size_t j = 0; j < shape.size(); ++j) {
        std::cout << shape[j];
        if (j != shape.size() - 1) {
          std::cerr << ", ";
        }
      }
      std::cout << "]" << std::endl;
    }

    for (int i = 0; i < this->onnx_tensor.sessionPtr->GetOutputCount(); i++) {
      auto outputnamePtr =
          this->onnx_tensor.sessionPtr->GetOutputNameAllocated(i, allocator);
      this->onnx_tensor.outputNodeNameAllocatedStrings.push_back(
          std::move(outputnamePtr));
      this->onnx_tensor.outputNames.push_back(
          this->onnx_tensor.outputNodeNameAllocatedStrings.back().get());
      std::cout << this->onnx_tensor.sessionPtr
                       ->GetOutputNameAllocated(i, allocator)
                       .get()
                << std::endl;
      this->onnx_tensor.outputShapes.push_back(
          this->onnx_tensor.sessionPtr->GetOutputTypeInfo(i)
              .GetTensorTypeAndShapeInfo()
              .GetShape());
      std::vector<int64_t> shape =
          this->onnx_tensor.sessionPtr->GetOutputTypeInfo(i)
              .GetTensorTypeAndShapeInfo()
              .GetShape();
      std::cerr << "Shape: [";
      for (size_t j = 0; j < shape.size(); ++j) {
        std::cout << shape[j];
        if (shape[j] > 0) {
          this->num_output_elements *= shape[j];
        }
        if (j != shape.size() - 1) {
          std::cerr << ", ";
        }
      }
      std::cout << "]" << std::endl;
    }
    std::cout << "[Loading] model successfully !!! " << std::endl;
  }

  // Updates internal clock inputs for periodic gait commands.
  void step_contact_targets(scalar_t frequencies) {
    const scalar_t phase = updatePhase(frequencies);
    const scalar_t sin_pos = std::sin(2.0 * M_PI * phase);
    const scalar_t cos_pos = std::cos(2.0 * M_PI * phase);

    this->clock_inputs[0] = sin_pos;
    this->clock_inputs[1] = cos_pos;
  }

  vector_t compute_observation(const Atom::BaseState& base_state,
                               const Atom::JointState& leg_state,
                               const Atom::ArmJointState& arm_state,
                               const Control& control, Observations& obs) {
    vector_t command(3);
    command << control.x, control.y, control.yaw;

    step_contact_targets(0.8);

    int num_of_policy_dofs = params->Get<int>("num_of_policy_dofs");
    int num_of_dofs = params->Get<int>("num_of_dofs");

    for (int i = 0; i < num_of_policy_dofs; i++) {
      obs.dof_pos[i] = leg_state.q[i];
      obs.dof_vel[i] = leg_state.dq[i];
    }
    for (int i = 0; i < num_of_dofs - num_of_policy_dofs; i++) {
      obs.dof_pos[i + num_of_policy_dofs] = arm_state.q[i];
      obs.dof_vel[i + num_of_policy_dofs] = arm_state.dq[i];
    }

    obs.base_quat.w() = base_state.qua[0];
    obs.base_quat.x() = base_state.qua[1];
    obs.base_quat.y() = base_state.qua[2];
    obs.base_quat.z() = base_state.qua[3];

    for (int i = 0; i < 3; i++) {
      obs.ang_vel[i] = base_state.w[i];
    }

    vector3_t baseEulerXyz = quatToXyz(obs.base_quat);

    vector_t deltaJointPos =
        obs.dof_pos.head(num_of_policy_dofs) -
        params->Get<vector_t>("default_dof_pos").head(num_of_policy_dofs);

    int num_one_step_observations =
        params->Get<int>("num_one_step_observations");

    vector_t proprioObs(num_one_step_observations);

    proprioObs << clock_inputs[0],
        clock_inputs[1],
        command.head(3).array() *
            params->Get<vector_t>("commands_scale").array(),
        deltaJointPos * params->Get<scalar_t>("dof_pos_scale"),
        obs.dof_vel.head(num_of_policy_dofs) *
            params->Get<scalar_t>("dof_vel_scale"),
        obs.actions, obs.ang_vel * params->Get<scalar_t>("ang_vel_scale"),
        baseEulerXyz;

    this->obs_history_buffer.head(this->obs_history_buffer.size() -
                                  num_one_step_observations) =
        this->obs_history_buffer.tail(this->obs_history_buffer.size() -
                                      num_one_step_observations);
    this->obs_history_buffer.tail(num_one_step_observations) = proprioObs;

    scalar_t obsMax = params->Get<scalar_t>("clip_obs");
    scalar_t obsMin = -1.0f * obsMax;
    this->obs_history_buffer =
        this->obs_history_buffer.array().max(obsMin).min(obsMax);

    return this->obs_history_buffer;
  }

  vector_t forward(const vector_t& observation) {
    // Create input tensor object.
    Ort::MemoryInfo memory_info =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    const tensor_element_t* input_ptr = observation.data();
    Ort::Value input_tensor = Ort::Value::CreateTensor<tensor_element_t>(
        memory_info, const_cast<tensor_element_t*>(input_ptr),
        observation.size(), onnx_tensor.inputShapes[0].data(),
        onnx_tensor.inputShapes[0].size());

    // Run inference.
    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues = onnx_tensor.sessionPtr->Run(
        runOptions, onnx_tensor.inputNames.data(), &input_tensor, 1,
        onnx_tensor.outputNames.data(), 1);

    vector_t result(this->num_output_elements);

    for (int i = 0; i < params->Get<int>("action_dim"); i++) {
      result[i] = static_cast<scalar_t>(
          *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i));
    }

    return result;
  }

  std::string name() const override { return "atom"; }
};
