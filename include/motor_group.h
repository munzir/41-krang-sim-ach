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

#include "ach_interface.h"  // InterfaceContext, Interface

class MotorGroupBase {
 public:
  enum MotorType { kSchunk = 0, kAmc, kUnlisted };
  enum MotorCommandType {
    kLock = 0,
    kPosition,
    kVelocity,
    kCurrent,
    kUnlock,
    kDoNothing
  };
  static MotorType FindMotorType(std::string& robot_joint_name,
                                 char* motor_config_file) {
    // The list of motor names corresponding to each motor type
    std::vector<std::string> motor_name_list = {"schunk", "amc"};

    // Get the motor name from the motor_config_file
    std::string motor_name;
    config4cpp::Configuration* cfg = config4cpp::Configuration::create();
    const char* scope = "";
    try {
      cfg->parse(motor_config_file);
      motor_name = std::string(
          cfg->lookupString(scope, (robot_joint_name + "_make").c_str()));
    } catch (const config4cpp::ConfigurationException& ex) {
      std::cerr << ex.c_str() << std::endl;
      cfg->destroy();
      return kUnlisted;
    }

    // Convert motor name to motor type and return the output
    for (int i = 0; i < motor_name_list.size(); i++) {
      if (!motor_name.compare(motor_name_list(i))) return (MotorType)i;
    }
    return kUnlisted;
  }

  MotorGroupBase() {}
  ~MotorGroup {}

  virtual void Run() {}
  virtual void Destroy() {}
}

template <class Motor>
class MotorGroup : public MotorGroupBase {
 public:
  MotorGroup(dart::dynamics::SkeletonPtr robot,
             InterfaceContext& interface_context,
             string::string& motor_group_name,
             std::vector<std::string>& motor_group_joints,
             std::string& motor_group_command_channel_name,
             std::string& motor_group_state_channel_name) {
    for (int i = 0; i < motor_group_joints.size(); i++) {
      motor_vector_.push_back(new Motor(robot, motor_group_joints[i]));
      command_val_.push_back(0.0);
    }
    interface_->Init(motor_vector_, interface_context, motor_group_name,
                     motor_group_command_channel_name,
                     motor_group_state_channel_name);
  }

  ~MotorGroup() { Destroy(); }

  void Run() {
    // Receive and execute command
    interface_->ReceiveCommand(&command, &command_val_);
    if(&command_ != kDoNothing) {
      Execute(command_, command_val_);
    }

    // Update and send state
    Update();
    interface_->SendState();
  }

  void Destroy() {
    for (int i = 0; i < motor_vector_.size(); i++) delete motor_vector_[i];
    interface_->Destroy();
  }

 private:
  std::vector<Motor> motor_vector_;
  Interface<Motor> interface_;

  MotorCommandType command_;
  std::vector<double> command_val_;
}
#endif  // KRANG_SIMULATION_MOTOR_GROUP_H_
