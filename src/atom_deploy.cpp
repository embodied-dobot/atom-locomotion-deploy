// Copyright (c) 2024-2025 Ziqi Fan
// Copyright (c) 2025 Shenzhen Yuejiang Technology Co., Ltd.
// SPDX-License-Identifier: Apache-2.0
// Modified by Dobot on 2025-12-1: design & configure parameters for Atom.

#include "atom_deploy.hpp"

#include "robot_fsm/fsm.hpp"
#include "robot_model/atom.hpp"

AtomDeploy::AtomDeploy() {
  // Read params from YAML.
  this->ReadYaml("config.yaml");
  this->fsm =
      std::make_shared<atom_deploy_fsm::AtomDeployFsm>(this, "StatePassive");

  // Init robot low-level commands and RL outputs.
  this->InitLowCmd();
  this->output_dof_pos = vector_t::Zero(Atom::kLegDofs);
  this->output_dof_vel = vector_t::Zero(Atom::kLegDofs);

  // Real-time control loop.
  this->loop_control = std::make_shared<LoopFunc>(
      "loop_control", this->params.Get<scalar_t>("dt"),
      [this]() { this->RobotControl(); });

  // RL inference loop.
  this->loop_rl = std::make_shared<LoopFunc>(
      "loop_rl", this->params.Get<scalar_t>("inference_dt"),
      [this]() { this->RunModel(); });

  this->loop_control->start();
  this->loop_rl->start();
}

AtomDeploy::~AtomDeploy() {
  this->loop_control->shutdown();
  this->loop_rl->shutdown();
  std::cout << "exit" << std::endl;
}

void AtomDeploy::GetState() {
  const std::shared_ptr<const Atom::BaseState> base_state_ptr =
      bridge.GetNewestBaseStatePtr();
  const std::shared_ptr<const Atom::JointState> js_tmp_ptr =
      bridge.GetNewestJointStatePtr();
  const std::shared_ptr<const RemoteCommand> remote_tmp_ptr =
      bridge.GetNewestRemoteCommandPtr();
  const std::shared_ptr<const Atom::ArmJointState> ajs_tmp_ptr =
      bridge.GetNewestArmStatePtr();

  this->base_state = *base_state_ptr;
  this->joint_state = *js_tmp_ptr;
  this->arm_joint_state = *ajs_tmp_ptr;

  if (remote_tmp_ptr->button_A_) {
    this->fsm_change = FsmChange::state_default_pos;
  } else if (remote_tmp_ptr->button_B_) {
    this->fsm_change = FsmChange::state_init_pose;
  } else if (remote_tmp_ptr->button_R1U_) {
    this->fsm_change = FsmChange::state_model;
  } else if (remote_tmp_ptr->button_SELECT_) {
    this->fsm_change = FsmChange::state_passive;
  }

  constexpr float kAlpha = 0.2f;
  float x_raw = remote_tmp_ptr->lin_vel[0] *
                this->params.Get<scalar_t>("max_speed_x");
  float y_raw = remote_tmp_ptr->lin_vel[1] *
                this->params.Get<scalar_t>("max_speed_y");
  float yaw_raw =
      remote_tmp_ptr->yaw_vel * this->params.Get<scalar_t>("max_yaw_rate");

  filtered_control.x = lowpass(x_raw, filtered_control.x, kAlpha);
  filtered_control.y = lowpass(y_raw, filtered_control.y, kAlpha);
  filtered_control.yaw =
      lowpass(yaw_raw, filtered_control.yaw, kAlpha);

  scalar_t deadzone = this->params.Get<scalar_t>("deadzone");

  if (filtered_control.x < deadzone && filtered_control.x > -deadzone) {
    filtered_control.x = 0.0;
  }
  if (filtered_control.y < deadzone && filtered_control.y > -deadzone) {
    filtered_control.y = 0.0;
  }
  if (filtered_control.yaw < deadzone && filtered_control.yaw > -deadzone) {
    filtered_control.yaw = 0.0;
  }

  this->control.x = filtered_control.x;
  this->control.y = filtered_control.y;
  this->control.yaw = filtered_control.yaw;
}

void AtomDeploy::SetCommand() {
  constexpr int kFsmId = 2;
  bridge.SetNewestFsmCommand(kFsmId);
  bridge.SetNewestLegCommand(this->leg_command);
  bridge.SetNewestArmCommand(this->arm_command);
}

void AtomDeploy::RobotControl() {
  {
    std::shared_lock<std::shared_mutex> lock(robot_state_mutex);
    this->GetState();
  }

  {
    std::unique_lock<std::shared_mutex> lock(robot_command_mutex);
    switch (this->fsm_change) {
      case FsmChange::state_default_pos:
        this->fsm->dispatch_event(atom_deploy_fsm::EventPolicyDefaultPos());
        break;
      case FsmChange::state_init_pose:
        this->fsm->dispatch_event(atom_deploy_fsm::EventInitPosExit());
        break;
      case FsmChange::state_model:
        this->fsm->dispatch_event(atom_deploy_fsm::EventRunningModel());
        break;
      case FsmChange::state_passive:
        this->fsm->dispatch_event(atom_deploy_fsm::EventPassive());
        break;
      case FsmChange::state_nochange:
      default:
        this->fsm->dispatch_event(atom_deploy_fsm::EventTick());
        break;
    }
  }

  this->fsm_change = FsmChange::state_nochange;

  {
    std::shared_lock<std::shared_mutex> lock(robot_command_mutex);
    this->SetCommand();
  }
}

void AtomDeploy::InitLowCmd() {
  this->leg_command.q_ref.setZero();
  this->leg_command.dq_ref.setZero();
  this->leg_command.kp.setZero();
  this->leg_command.kd.setZero();
  this->leg_command.tau_forward.setZero();

  this->arm_command.q_ref.setZero();
  this->arm_command.dq_ref.setZero();
  this->arm_command.kp.setZero();
  this->arm_command.kd.setZero();
  this->arm_command.tau_forward.setZero();
}

void AtomDeploy::ReadYaml(const std::string& file_name) {
  std::string config_path = std::string(POLICY_DIR) + "/" + file_name;
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path)["atom"];
  } catch (YAML::BadFile& e) {
    throw std::runtime_error("The file '" + config_path + "' does not exist");
  }

  for (auto it = config.begin(); it != config.end(); ++it) {
    std::string key = it->first.as<std::string>();
    this->params.config_node[key] = it->second;
  }
}


void AtomDeploy::InitRL() {

  this->obs.ang_vel.setZero();
  this->obs.gravity_vec << 0.0, 0.0, -1.0;
  this->obs.base_quat = quaternion_t(1.0, 0.0, 0.0, 0.0);
  this->obs.dof_pos = this->params.Get<vector_t>("default_dof_pos");
  this->obs.dof_vel = vector_t::Zero(Atom::kLegDofs + Atom::kArmDofs);
  this->obs.actions = vector_t::Zero(Atom::kLegDofs);

  // Init model.
  this->active_model = std::make_unique<ATOMModel>(this->params);
}

void AtomDeploy::RunModel() {
  if (this->rl_init_done) {
    this->Forward();
  }
}

void AtomDeploy::Forward() {
  auto policy_obs = this->active_model->compute_observation(
      this->base_state, this->joint_state, this->arm_joint_state,
      this->control, this->obs);

  this->obs.actions = this->active_model->forward(policy_obs);

  scalar_t actionMax = this->params.Get<scalar_t>("clip_actions");
  scalar_t actionMin = -1.0f * actionMax;
  this->obs.actions =
      this->obs.actions.array().max(actionMin).min(actionMax);

  vector_t actions_scaled =
      this->obs.actions.array() *
      this->params.Get<vector_t>("action_scale").array();
  this->output_dof_pos =
      actions_scaled +
      this->params.Get<vector_t>("default_dof_pos")
          .head(this->params.Get<int>("action_dim"));
  this->output_dof_vel = 0.0 * this->obs.dof_vel;

  this->output_dof_pos_queue.push(this->output_dof_pos);
  this->output_dof_vel_queue.push(this->output_dof_vel);
}


volatile sig_atomic_t g_shutdown_requested = 0;
void signalHandler(int signum) {
  std::cout << "Received signal " << signum << ", shutting down..."
            << std::endl;
  g_shutdown_requested = 1;
}

int main(int argc, char **argv) {
  signal(SIGINT, signalHandler);

  AtomDeploy atom_deploy;
  while (!g_shutdown_requested) {
    sleep(1);
  }

  std::cout << "AtomDeploy exit" << std::endl;
  return 0;
}
