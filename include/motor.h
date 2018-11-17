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
 * @file motor.h
 * @author Munzir Zafar
 * @date Nov 15, 2018
 * @brief Header for motor.cpp that handles ach communication for a specific
 * motor group
 */

#ifndef KRANG_SIMULATION_MOTOR_H_
#define KRANG_SIMULATION_MOTOR_H_

#include <vector>

#include <ach.h>
#include <somatic.h>
#include <somatic.pb-c.h>

class MotorGroup {
 public:
  MotorGroup(const somatic_d_t& daemon, const char* cmd_chan_name,
             const char* state_chan_name, const std::vector<int> joint_indices,
             const std::vector<double>& sign = std::vector<double>());
  ~MotorGroup();

  void SendState(const std::vector<double>& all_pos,
                 const std::vector<double>& all_vel,
                 const std::vector<double>& all_cur);

 private:
  void InitMessage();

  somatic_d_t* daemon_;
  size_t n_;  // module count
  ach_channel_t cmd_chan_;
  ach_channel_t state_chan_;
  std::vector<int> joint_indices_;
  std::vector<double> sign_;
  Somatic__MotorState state_msg_;
  struct {
    Somatic__Vector position_;
    Somatic__Vector velocity_;
    Somatic__Vector current_;
  } state_msg_fields_;
};

#endif  // KRANG_SIMULATION_MOTOR_H_
