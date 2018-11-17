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

KrangAch::KrangAch(SimConfig& params) {
  memset(&daemon_opts_, 0, sizeof(daemon_opts_));
  daemon_opts_.ident = strdup(params.daemonIdentifier);
  daemon_opts_.daemonize = params.daemonize;
  daemon_opts_.sched_rt = SOMATIC_D_SCHED_MOTOR;
  somatic_verbprintf_prefic = daemon_opts_.ident;
  somatic_d_init(&daemon_, &daemon_opts_);

  std::vector<int> wheels_joint_indices = {6, 7};
  wheels_ = MotorGroup(daemon_, params.wheelsCmdChan, params.wheelsStateChan,
                       wheels_joint_indices);

  std::vector<int> waist_joint_indices = {8, 8};
  std::vector<double> waist_joint_sign = {1.0, -1.0};
  waist_ = MotorGroup(daemon_, params.waistCmdChan, params.waistStateChan,
                      waist_joint_indices, waist_joint_sign);

  std::vector<int> torso_joint_indices = {9};
  torso_ = MotorGroup(daemon_, params.torsoCmdChan, params.torsoStateChan,
                      torso_joint_indices);

  std::vector<int> left_arm_joint_indices = {10, 11, 12, 13, 14, 15, 16};
  left_arm_ = MotorGroup(daemon_, params.leftArmCmdChan,
                         params.leftArmStateChan, left_arm_joint_indices);

  std::vector<int> right_arm_joint_indices = {17, 18, 19, 20, 21, 22, 23};
  right_arm_ = MotorGroup(daemon_, params.rightArmCmdChan,
                          params.rightArmStateChan, right_arm_joint_indices);
}
