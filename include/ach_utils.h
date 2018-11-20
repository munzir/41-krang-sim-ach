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
 * @file ach_utils.h
 * @author Munzir Zafar
 * @date Nov 15, 2018
 * @brief Header for ach_utils.cpp that uses MotorGroup class to perform ach
 * communication of krang
 */

#ifndef KRANG_SIMULATION_KRANG_ACH_H_
#define KRANG_SIMULATION_KRANG_ACH_H_

#include <memory>

#include <somatic.h>

#include "motor.h"
#include "sim_config.h"

class KrangAch {
 public:
  KrangAch(SimConfig& params);
  ~KrangAch(){};

  // Since special cleaning up is needed and glutMainLoop exits abruptly by
  // calling exit(0) that does not call destructors, the cleaning that could
  // elegantly be done in the destructor has to be explicitly performed now in a
  // separate Destroy() function that the user needs to call in an ExitFunction
  // that can be specified to be executed via atexit(ExitFunction)
  void Destroy();

  void SendState(const Eigen::VectorXd& all_pos, const Eigen::VectorXd& all_vel,
                 const Eigen::VectorXd& all_cur,
                 const Eigen::Matrix<double, 6, 1>& imu_data);

  // For somatic daemon management
  somatic_d_t daemon_;
  somatic_d_opts_t daemon_opts_;

  // MotorGroups for ach communication
  MotorGroup *wheels_, *waist_, *torso_, *left_arm_, *right_arm_;

  // Imu data ach communication
  ach_channel_t imu_chan_;
  Somatic__Vector* imu_msg_;
};

#endif  // KRANG_SIMULATION_KRANG_ACH_H_
