// Copyright (c) 2025 Shenzhen Yuejiang Technology Co., Ltd.
// SPDX-License-Identifier: Apache-2.0

// Lightweight FSM helper and concrete AtomDeploy FSM implementation.
#pragma once

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "utils/utils.hpp"

class AtomDeploy;

namespace atom_deploy_fsm {

// Generic CRTP-based FSM wrapper.
template <typename Derived, typename StateVariant>
class Fsm {
 public:
  const StateVariant& get_state() const { return state_; }
  StateVariant& get_state() { return state_; }

  template <typename Event>
  void dispatch(Event&& event) {
    Derived& child = static_cast<Derived&>(*this);
    auto new_state = std::visit(
        [&](auto& s) -> std::optional<StateVariant> {
          return child.on_event(s, std::forward<Event>(event));
        },
        state_);
    if (new_state) {
      state_ = *std::move(new_state);
    }
  }

 private:
  StateVariant state_;
};

// Events.
struct EventTick {};
struct EventPolicyDefaultPos {};
struct EventInitPosExit {};
struct EventPassive {};
struct EventRunningModel {};

// States.
struct StatePassive {};
struct StateDefaultPos {
  float percent = 0.0f;
  std::chrono::steady_clock::time_point start_time;
};
struct StateInitPose {
  float percent = 0.0f;
  std::chrono::steady_clock::time_point start_time;
};
struct StateModel {
  float percent = 0.0f;
  bool rl_init_done = false;
};

using AtomState =
    std::variant<StatePassive, StateDefaultPos, StateInitPose, StateModel>;

// FSM driving AtomDeploy control modes.
class AtomDeployFsm : public Fsm<AtomDeployFsm, AtomState> {
 public:
  AtomDeploy* deploy;

  explicit AtomDeployFsm(AtomDeploy* d,
                         const std::string& initial = "StatePassive")
      : deploy(d) {
    if (initial == "StateDefaultPos") {
      this->get_state() = StateDefaultPos{};
    } else if (initial == "StateInitPose") {
      this->get_state() = StateInitPose{};
    } else if (initial == "StateModel") {
      this->get_state() = StateModel{};
    } else {
      this->get_state() = StatePassive{};
    }

    call_enter_for_state(this->get_state());
  }

  template <typename Event>
  void dispatch_event(const Event& ev) {
    size_t prev_index = this->get_state().index();
    this->dispatch(ev);
    size_t new_index = this->get_state().index();
    if (prev_index != new_index) {
      call_exit_by_index(prev_index);
      call_enter_for_state(this->get_state());
    }
  }

  template <typename StateT, typename EventT>
  auto on_event(StateT&, const EventT&) {
    return std::optional<AtomState>{};
  }

  // ----- Passive state -----
  auto on_event(StatePassive&, const EventTick&) {
    if (!deploy) {
      return std::optional<AtomState>{};
    }

    for (int i = 0; i < 12; ++i) {
      deploy->leg_command.q_ref[i] = deploy->joint_state.q[i];
      deploy->leg_command.dq_ref[i] = 0.0;
      deploy->leg_command.kp[i] = 0.0;
      deploy->leg_command.kd[i] = 2.0;
      deploy->leg_command.tau_forward[i] = 0.0;
    }

    for (int i = 0; i < 17; ++i) {
      deploy->arm_command.q_ref[i] = deploy->arm_joint_state.q[i];
      deploy->arm_command.dq_ref[i] = 0.0;
      deploy->arm_command.kp[i] = 0.0;
      deploy->arm_command.kd[i] = 2.0;
      deploy->arm_command.tau_forward[i] = 0.0;
    }

    return std::optional<AtomState>{};
  }

  auto on_event(StatePassive&, const EventPolicyDefaultPos&) {
    std::cout
        << "[on_event] current in passive mode, changing to default_pos mode"
        << std::endl;
    return AtomState{StateDefaultPos{}};
  }

  // ----- Default-pos state -----
  auto on_event(StateDefaultPos& s, const EventTick&) {
    if (!deploy) {
      return std::optional<AtomState>{};
    }

    constexpr float kDurationSeconds = 2.0f;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now - s.start_time)
                       .count() /
                   1000.0f;

    s.percent = std::min(1.0f, elapsed / kDurationSeconds);

    auto default_pos =
        deploy->params.Get<std::vector<scalar_t>>("default_dof_pos");
    auto fixed_kp = deploy->params.Get<vector_t>("fixed_kp");
    auto fixed_kd = deploy->params.Get<vector_t>("fixed_kd");

    for (int i = 0; i < 12; ++i) {
      scalar_t q_target = default_pos[i];
      scalar_t q_current = deploy->now_joint_state.q[i];
      float t = s.percent;
      deploy->leg_command.q_ref[i] = (1.0f - t) * q_current + t * q_target;
      deploy->leg_command.dq_ref[i] = 0.0;
      deploy->leg_command.kp[i] = fixed_kp[i];
      deploy->leg_command.kd[i] = fixed_kd[i];
      deploy->leg_command.tau_forward[i] = 0.0;
    }

    for (int i = 0; i < 17; ++i) {
      scalar_t q_current = deploy->now_arm_joint_state.q[i];
      float t = s.percent;
      deploy->arm_command.q_ref[i] =
          (1.0f - t) * q_current + t * static_cast<scalar_t>(0.0);
      deploy->arm_command.dq_ref[i] = 0.0;
      deploy->arm_command.kp[i] = fixed_kp[i + 12];
      deploy->arm_command.kd[i] = fixed_kd[i + 12];
      deploy->arm_command.tau_forward[i] = 0.0;
    }

    return std::optional<AtomState>{};
  }

  auto on_event(StateDefaultPos&, const EventPassive&) {
    std::cout
        << "[on_event] current in default_pos mode, changing to passive mode"
        << std::endl;
    return AtomState{StatePassive{}};
  }

  auto on_event(StateDefaultPos&, const EventRunningModel&) {
    std::cout
        << "[on_event] current in default_pos mode, changing to rl_model mode"
        << std::endl;
    return AtomState{StateModel{}};
  }

  auto on_event(StateDefaultPos&, const EventInitPosExit&) {
    std::cout
        << "[on_event] current in default_pos mode, changing to init_pose mode"
        << std::endl;
    return AtomState{StateInitPose{}};
  }

  // ----- Init-pose state -----
  auto on_event(StateInitPose& s, const EventTick&) {
    if (!deploy) {
      return std::optional<AtomState>{};
    }

    constexpr float kDurationSeconds = 2.0f;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now - s.start_time)
                       .count() /
                   1000.0f;

    s.percent = std::min(1.0f, elapsed / kDurationSeconds);

    auto fixed_kp = deploy->params.Get<vector_t>("fixed_kp");
    auto fixed_kd = deploy->params.Get<vector_t>("fixed_kd");

    for (int i = 0; i < 12; ++i) {
      scalar_t q_target = deploy->start_joint_state.q[i];
      scalar_t q_current = deploy->now_joint_state.q[i];
      float t = s.percent;
      deploy->leg_command.q_ref[i] = (1.0f - t) * q_current + t * q_target;
      deploy->leg_command.dq_ref[i] = 0.0;
      deploy->leg_command.kp[i] = fixed_kp[i];
      deploy->leg_command.kd[i] = fixed_kd[i];
      deploy->leg_command.tau_forward[i] = 0.0;
    }

    for (int i = 0; i < 17; ++i) {
      scalar_t q_target = deploy->start_arm_joint_state.q[i];
      scalar_t q_current = deploy->now_arm_joint_state.q[i];
      float t = s.percent;
      deploy->arm_command.q_ref[i] = (1.0f - t) * q_current + t * q_target;
      deploy->arm_command.dq_ref[i] = 0.0;
      deploy->arm_command.kp[i] = fixed_kp[i + 12];
      deploy->arm_command.kd[i] = fixed_kd[i + 12];
      deploy->arm_command.tau_forward[i] = 0.0;
    }

    if (s.percent >= 1.0f) {
      return std::optional<AtomState>(std::in_place_type<StatePassive>);
    }

    return std::optional<AtomState>{};
  }

  auto on_event(StateInitPose&, const EventPolicyDefaultPos&) {
    std::cout
        << "[on_event] current in init_pose mode, changing to default_pos mode"
        << std::endl;
    return AtomState{StateDefaultPos{}};
  }

  // ----- Model state -----
  auto on_event(StateModel& s, const EventTick&) {
    if (!deploy) {
      return std::optional<AtomState>{};
    }

    if (!s.rl_init_done) {
      try {
        deploy->InitRL();
        deploy->now_joint_state = deploy->joint_state;
        s.rl_init_done = true;
        deploy->rl_init_done = true;
        std::cout << "Entered Model mode (RL control)" << std::endl;
      } catch (const std::exception& e) {
        std::cout << "InitRL() failed: " << e.what() << std::endl;
        s.rl_init_done = false;
        deploy->rl_init_done = false;
      }
    }

    if (!s.rl_init_done) {
      return std::optional<AtomState>{};
    }

    const int action_dim = deploy->params.Get<int>("action_dim");
    (void)action_dim;  // Currently unused but kept for clarity.

    auto rl_kp = deploy->params.Get<vector_t>("rl_kp");
    auto rl_kd = deploy->params.Get<vector_t>("rl_kd");

    auto fixed_kp = deploy->params.Get<vector_t>("fixed_kp");
    auto fixed_kd = deploy->params.Get<vector_t>("fixed_kd");

    vector_t output_dof_pos;
    vector_t output_dof_vel;
    if (deploy->output_dof_pos_queue.try_pop(output_dof_pos) &&
        deploy->output_dof_vel_queue.try_pop(output_dof_vel)) {
      for (int i = 0; i < 12; ++i) {
        deploy->leg_command.q_ref[i] = output_dof_pos[i];
        deploy->leg_command.dq_ref[i] = output_dof_vel[i];
        deploy->leg_command.kp[i] = rl_kp[i];
        deploy->leg_command.kd[i] = rl_kd[i];
        deploy->leg_command.tau_forward[i] = 0.0;
      }
    }

    for (int i = 0; i < 17; ++i) {
      deploy->arm_command.q_ref[i] =
          deploy->params.Get<vector_t>("default_dof_pos")[i + 12];
      deploy->arm_command.dq_ref[i] = 0.0;
      deploy->arm_command.kp[i] = fixed_kp[i + 12];
      deploy->arm_command.kd[i] = fixed_kd[i + 12];
      deploy->arm_command.tau_forward[i] = 0.0;
    }

    return std::optional<AtomState>{};
  }

  auto on_event(StateModel&, const EventPassive&) {
    std::cout << "[on_event] current in model mode, changing to passive mode"
              << std::endl;
    return AtomState{StatePassive{}};
  }

  auto on_event(StateModel&, const EventPolicyDefaultPos&) {
    std::cout
        << "[on_event] current in model mode, changing to default_pos mode"
        << std::endl;
    return AtomState{StateDefaultPos{}};
  }

  auto on_event(StateModel&, const EventInitPosExit&) {
    std::cout
        << "[on_event] current in model mode, changing to init_pose mode"
        << std::endl;
    return AtomState{StateInitPose{}};
  }

 private:
  void call_enter_for_state(const AtomState& st) {
    std::visit(
        [this](auto const& s) {
          using T = std::decay_t<decltype(s)>;
          if constexpr (std::is_same_v<T, StatePassive>) {
            std::cout << "Entered Passive mode (fsm)" << std::endl;
          } else if constexpr (std::is_same_v<T, StateDefaultPos>) {
            auto& sref = const_cast<StateDefaultPos&>(s);
            sref.percent = 0.0f;
            sref.start_time = std::chrono::steady_clock::now();
            if (deploy) {
              deploy->now_joint_state = deploy->joint_state;
              deploy->now_arm_joint_state = deploy->arm_joint_state;
            }
            std::cout << "Entered DefaultPos mode (fsm)" << std::endl;
          } else if constexpr (std::is_same_v<T, StateInitPose>) {
            auto& sref = const_cast<StateInitPose&>(s);
            sref.percent = 0.0f;
            sref.start_time = std::chrono::steady_clock::now();
            if (deploy) {
              deploy->now_joint_state = deploy->joint_state;
              deploy->now_arm_joint_state = deploy->arm_joint_state;
            }
            std::cout << "Entered InitPose mode (fsm)" << std::endl;
          } else if constexpr (std::is_same_v<T, StateModel>) {
            auto& sref = const_cast<StateModel&>(s);
            sref.percent = 0.0f;
            sref.rl_init_done = false;
            std::cout << "Entering Model mode (fsm) - will init on tick"
                      << std::endl;
          }
        },
        st);
  }

  void call_exit_by_index(size_t idx) {
    // variant index mapping:
    // 0: StatePassive
    // 1: StateDefaultPos
    // 2: StateInitPose
    // 3: StateModel
    switch (idx) {
      case 0:
        // passive Exit() - nothing to do
        break;
      case 1:
        // default pos Exit
        break;
      case 2:
        // init pose Exit
        break;
      case 3:
        // model Exit -> ensure RL done flag cleared
        if (deploy) {
          std::cout << "[call_exit_by_index] current in model mode, clearing "
                       "rl_init_done flag"
                    << std::endl;
          deploy->rl_init_done = false;
        }
        break;
      default:
        break;
    }
  }
};

}  // namespace atom_deploy_fsm
