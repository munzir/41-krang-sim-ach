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

#include <config4cpp/Configuration.h>  // config4cpp::Configuration
#include <dart/dart.hpp>               // dart::dynamics
#include <string>                      // std::string
#include <vector>                      // std::vector

#include "ach_interface.h"  // InterfaceContext, SchunkMotorInterface, AmcMotorInterface
#include "motor.h"          // MotorBase::, SchunkMotor, AmcMotor

class MotorGroup {
 public:
  enum MotorCommandType {
    kLock = 0,
    kPosition,
    kVelocity,
    kCurrent,
    kUnlock,
    kDoNothing
  };
  MotorGroup(dart::dynamics::SkeletonPtr robot,
             InterfaceContext& interface_context,
             string::string& motor_group_name,
             std::vector<std::string>& motor_group_joints,
             const* motor_config_file,
             std::string& motor_group_command_channel_name,
             std::string& motor_group_state_channel_name) {
    switch (
        MotorBase::FindMotorType(motor_group_joints[i][0], motor_config_file)) {
      case MotorBase::kSchunk: {
        for (int i = 0; i < motor_group_joints.size(); i++) {
          motor_vector_.push_back(
              new SchunkMotor(robot, motor_group_joints[i]));
          command_val_.push_back(0.0);
        }
        interface_ = new SchunkMotorInterface(
            motor_vector_, interface_context, motor_group_name,
            motor_group_command_channel_name, motor_group_state_channel_name);
        break;
      }
      case MotorBase::kAmc: {
        for (int i = 0; i < motor_group_joints.size(); i++) {
          motor_vector_.push_back(new AmcMotor(robot, motor_group_joints[i]));
          command_val_.push_back(0.0);
        }
        interface_ = new AmcMotorInterface(
            motor_vector_, interface_context, motor_group_name,
            motor_group_command_channel_name, motor_group_state_channel_name);
        break;
      }
      case MotorBase::kUnlisted: {
        assert(false && "Error Identifying motor type");
        break;
      }
    }
  }

  ~MotorGroup() { Destroy(); }

  void Update() {
    for (int i = 0; i < motor_vector_.size(); i++) motor_vector_[i]->Update();
  }

  void Execute(MotorCommandType& command, std::vector<double>& command_val) {
    switch (command) {
      case kLock: {
        for (int i = 0; i < motor_vector_.size(); i++) motor_vector_[i]->Lock();
        break;
      }
      case kUnlock: {
        for (int i = 0; i < motor_vector_.size(); i++)
          motor_vector_[i]->Unlock();
        break;
      }
      case kPosition: {
        for (int i = 0; i < motor_vector_.size(); i++)
          motor_vector_[i]->PositionCmd(command_vals[i]);
        break;
      }
      case kVelocity: {
        for (int i = 0; i < motor_vector_.size(); i++)
          motor_vector_[i]->VelocityCmd(command_vals[i]);
        break;
      }
      case kCurrent: {
        for (int i = 0; i < motor_vector_.size(); i++)
          motor_vector_[i]->CurrentCmd(command_vals[i]);
        break;
      }
      case kDoNothing: {
        break;
      }
    }
  }

  void Run() {
    // Receive and execute command
    interface_->ReceiveCommand(&command_, &command_val_);
    if (command_ != kDoNothing) {
      Execute(command_, command_val_);
    }

    // Update and send state
    Update();
    interface_->SendState();
  }

  void Destroy() {
    for (int i = 0; i < motor_vector_.size(); i++) delete motor_vector_[i];
    motor_vector_.clear();
    command_val_.clear();
    interface_->Destroy();
  }

 private:
  std::vector<MotorBase*> motor_vector_;
  MotorInterfaceBase* interface_;

  MotorCommandType command_;
  std::vector<double> command_val_;
}
#endif  // KRANG_SIMULATION_MOTOR_GROUP_H_
