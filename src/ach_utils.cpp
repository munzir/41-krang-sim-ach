/*
 * Copyright (c) 2018, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file ach_utils.cpp
 * @author Munzir Zafar
 * @date Nov 15, 2018
 * @brief Uses MotorGroup class to perform ach communication of krang
 */

#include "ach_utils.h"

#include <vector>

#include <somatic.h>
#include <somatic/daemon.h>
#include "motor.h"
#include "sim_config.h"

KrangAch::KrangAch(SimConfig& params) {
  // Initialize the daemon_
  memset(&daemon_opts_, 0, sizeof(daemon_opts_));
  daemon_opts_.ident = strdup(params.daemon_identifier);
  daemon_opts_.daemonize = params.daemonize;
  daemon_opts_.sched_rt = SOMATIC_D_SCHED_MOTOR;
  somatic_verbprintf_prefix = daemon_opts_.ident;
  memset(&daemon_, 0, sizeof(daemon_));
  somatic_d_init(&daemon_, &daemon_opts_);

  // Establish ach channel communication for all motor groups
  // TODO: Get indices from joint labels in urdf
  std::vector<int> wheels_joint_indices = {6, 7};
  wheels_ = MotorGroup("wheels", &daemon_, params.wheels_cmd_chan,
                       params.wheels_state_chan, wheels_joint_indices);

  std::vector<int> waist_joint_indices = {8, 8};
  std::vector<double> waist_joint_sign = {1.0, -1.0};
  waist_ = MotorGroup("waist", &daemon_, params.waist_cmd_chan,
                      params.waist_state_chan, waist_joint_indices,
                      waist_joint_sign);

  std::vector<int> torso_joint_indices = {9};
  torso_ = MotorGroup("torso", &daemon_, params.torso_cmd_chan,
                      params.torso_state_chan, torso_joint_indices);

  std::vector<int> left_arm_joint_indices = {11, 12, 13, 14, 15, 16, 17};
  left_arm_ = MotorGroup("left-arm", &daemon_, params.left_arm_cmd_chan,
                         params.left_arm_state_chan, left_arm_joint_indices);

  std::vector<int> right_arm_joint_indices = {18, 19, 20, 21, 22, 23, 24};
  right_arm_ = MotorGroup("right-arm", &daemon_, params.right_arm_cmd_chan,
                          params.right_arm_state_chan, right_arm_joint_indices);

  // Send a "running" notice on the event channel
  somatic_d_event(&daemon_, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

KrangAch::~KrangAch() {
  // Send a "stopping" notice on the event channel
  somatic_d_event(&daemon_, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

  // Destroy the daemon
  somatic_d_destroy(&daemon_);
}
void KrangAch::SendState(const Eigen::VectorXd& all_pos,
                         const Eigen::VectorXd& all_vel,
                         const Eigen::VectorXd& all_cur) {
  wheels_.SendState(all_pos, all_vel, all_cur);
  waist_.SendState(all_pos, all_vel, all_cur);
  torso_.SendState(all_pos, all_vel, all_cur);
  left_arm_.SendState(all_pos, all_vel, all_cur);
  right_arm_.SendState(all_pos, all_vel, all_cur);
}
