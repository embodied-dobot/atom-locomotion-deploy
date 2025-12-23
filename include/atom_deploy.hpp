// Copyright (c) 2025 Shenzhen Yuejiang Technology Co., Ltd.
// SPDX-License-Identifier: Apache-2.0

// Main deployment entry for RL-controlled Atom robot.
// Provides robot state access, FSM management and RL model forward loop.
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <shared_mutex>
#include <string>
#include <unistd.h>  // NOLINT

#include <Eigen/Dense>
#include <tbb/concurrent_queue.h>

#include "common/bridge.h"
#include "common/joystick.h"
#include "common/motor_command.h"

#include "loop/loop.hpp"
#include "utils/utils.hpp"

namespace atom_deploy_fsm {
    class AtomDeployFsm;
}

enum class FsmChange {
    state_nochange,
    state_passive,
    state_default_pos,
    state_init_pose,
    state_model
};

class AtomDeploy {
public:
    AtomDeploy();
    ~AtomDeploy();

    YamlParams params;
    Observations obs;
    Control control;
    ControlFiltered filtered_control;

    RemoteCommand remote_command;
    Atom::Bridge bridge;
    Atom::LegCommand leg_command;
    Atom::ArmCommand arm_command;

    Atom::BaseState base_state;
    Atom::JointState joint_state;
    Atom::ArmJointState arm_joint_state; 
    Atom::JointState start_joint_state;
    Atom::ArmJointState start_arm_joint_state;
    Atom::JointState now_joint_state;
    Atom::ArmJointState now_arm_joint_state;
    bool rl_init_done = false;

    std::shared_ptr<atom_deploy_fsm::AtomDeployFsm> fsm;
    FsmChange fsm_change = FsmChange::state_nochange;

  // Initializes low-level command structures to safe zeros.
    void InitLowCmd();

  // Initializes RL-related buffers and model.
    void InitRL();

  // Main high-level control loop: reads state, runs FSM and sends commands.
    void RobotControl();

  // Reads latest robot and remote-control state from the bridge.
  void GetState();

  // Writes latest leg / arm command and FSM command back to the bridge.
    void SetCommand();

  // Runs RL model once if initialization is finished.
    void RunModel();

  // Performs a full forward pass of the RL policy and updates output queues.
    void Forward();

    std::unique_ptr<RobotModel> active_model;

  // Loads YAML parameters from policy directory.
  void ReadYaml(const std::string& file_name);

    // loop
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;

    // output buffer
    vector_t output_dof_pos;
    vector_t output_dof_vel;
    tbb::concurrent_queue<vector_t> output_dof_pos_queue;
    tbb::concurrent_queue<vector_t> output_dof_vel_queue;
    
    // thread safety
    mutable std::shared_mutex robot_state_mutex;
    mutable std::shared_mutex robot_command_mutex;

};