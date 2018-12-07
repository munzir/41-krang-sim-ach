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
 * @file motor_group.cpp
 * @author Munzir Zafar
 * @date Nov 26, 2018
 * @brief MotorGroup class is a composite of a colloection of Motor objects and
 * one Interface object. Allows interface to control the motors.
 */

#include "motor_group.h"

#include "ach_interface.h"  // InterfaceContext, interface::Create
#include "motor_base.h"     // motor::Create

#include <dart/dart.hpp>  // dart::dynamics
#include <string>         // std::string
#include <thread>         // std::thread
#include <vector>         // std::vector

MotorGroup::MotorGroup(dart::dynamics::SkeletonPtr robot,
                       InterfaceContext& interface_context,
                       std::string& motor_group_name,
                       std::vector<std::string>& motor_group_joints,
                       const char* motor_config_file,
                       std::string& motor_group_command_channel_name,
                       std::string& motor_group_state_channel_name) {
  for (int i = 0; i < motor_group_joints.size(); i++) {
    motor_vector_.push_back(
        motor::Create(robot, motor_group_joints[i], motor_config_file));
    command_val_.push_back(0.0);
  }
  interface_ = interface::Create(
      motor_vector_, interface_context, motor_group_name,
      motor_group_command_channel_name, motor_group_state_channel_name);

  thread_ = new std::thread(&MotorGroup::InfiniteRun, this);
}

void MotorGroup::Update() {
  for (int i = 0; i < motor_vector_.size(); i++) motor_vector_[i]->Update();
}

void MotorGroup::Execute(MotorBase::MotorCommandType& command,
                         std::vector<double>& command_val) {
  switch (command) {
    case MotorBase::kLock: {
      for (int i = 0; i < motor_vector_.size(); i++) motor_vector_[i]->Lock();
      break;
    }
    case MotorBase::kUnlock: {
      for (int i = 0; i < motor_vector_.size(); i++) motor_vector_[i]->Unlock();
      break;
    }
    case MotorBase::kPosition: {
      for (int i = 0; i < motor_vector_.size(); i++)
        motor_vector_[i]->PositionCmd(command_val[i]);
      break;
    }
    case MotorBase::kVelocity: {
      for (int i = 0; i < motor_vector_.size(); i++)
        motor_vector_[i]->VelocityCmd(command_val[i]);
      break;
    }
    case MotorBase::kCurrent: {
      for (int i = 0; i < motor_vector_.size(); i++)
        motor_vector_[i]->CurrentCmd(command_val[i]);
      break;
    }
    case MotorBase::kDoNothing: {
      break;
    }
  }
}

void MotorGroup::Run() {
  // Receive and execute command
  interface_->ReceiveCommand(&command_, &command_val_);
  if (command_ != MotorBase::kDoNothing) {
    Execute(command_, command_val_);
  }

  // Update and send state
  Update();
  interface_->SendState();
}

void MotorGroup::Destroy() {
  delete thread_;
  for (int i = 0; i < motor_vector_.size(); i++) delete motor_vector_[i];
  motor_vector_.clear();
  command_val_.clear();
  interface_->Destroy();
}
