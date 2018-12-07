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
 * @file motor_group.h
 * @author Munzir Zafar
 * @date Nov 24, 2018
 * @brief MotorGroup class is a composite of a colloection of Motor objects and
 * one Interface object. Allows interface to control the motors.
 */

#ifndef KRANG_SIMULATION_MOTOR_GROUP_H_
#define KRANG_SIMULATION_MOTOR_GROUP_H_

#include "ach_interface.h"  // InterfaceContext, SchunkMotorInterface, AmcMotorInterface
#include "motor_base.h"  // MotorBase::, SchunkMotor, AmcMotor

#include <dart/dart.hpp>  // dart::dynamics
#include <string>         // std::string
#include <thread>         // std::thread
#include <vector>         // std::vector

class MotorGroup {
 public:
  MotorGroup(dart::dynamics::SkeletonPtr robot,
             InterfaceContext& interface_context, std::string& motor_group_name,
             std::vector<std::string>& motor_group_joints,
             const char* motor_config_file,
             std::string& motor_group_command_channel_name,
             std::string& motor_group_state_channel_name);

  ~MotorGroup() { Destroy(); }

  void Update();

  void Execute(MotorBase::MotorCommandType& command,
               std::vector<double>& command_val);

  void Run();

  void InfiniteRun() {
    while (true) Run();
  }

  void Destroy();

 private:
  std::vector<MotorBase*> motor_vector_;
  MotorInterfaceBase* interface_;

  MotorBase::MotorCommandType command_;
  std::vector<double> command_val_;

  std::thread* thread_;
};
#endif  // KRANG_SIMULATION_MOTOR_GROUP_H_
