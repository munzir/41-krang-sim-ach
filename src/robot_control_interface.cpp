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
 * @file robot_control_interface.cpp
 * @author Munzir Zafar
 * @date Nov 23, 2018
 * @brief RobotControlInterface class that is a composite of MotorGroup and
 * SensorGroup objects to allow external interface to access robot states and
 * control robot actuators
 */

#include "robot_control_interface.h"

#include <assert.h>                    // assert()
#include <config4cpp/Configuration.h>  // config4cpp::Configuration
#include <stdio.h>                     // std::cout
#include <dart/dart.hpp>               // dart::dynamics
#include <iterator>                    // std::istream_iterator
#include <sstream>                     // std::stringstream
#include <string>                      // std::string

#include "motor_group.h"   // FindMotorType, MotorGroup()
#include "sensor_group.h"  // FindSensorType, SensorGroup()

RobotControlInterface::RobotControlInterface(dart::dynamics::SkeletonPtr robot,
                                             char* motor_config_file,
                                             char* interface_config_file) {
  interface_context_ = new InterfaceContext(interface_config_file);

  RobotControlInterfaceParams params;
  ReadParams(interface_config_file, &params);

  for (int i = 0; i < params.num_sensor_gorups; i++) {
    sensor_groups_.push_back(new SensorGroup(
        robot, interface_context_, params.sensor_group_names_[i],
        params.sensor_group_state_channel_names_[i]));
  }
  for (int i = 0; i < params.num_motor_groups; i++) {
    motor_groups_.push_back(
        new MotorGroup(robot, interface_context_, params.motor_group_names_[i],
                       params.motor_group_joints_[i], motor_config_file,
                       params.motor_group_command_channel_names_[i],
                       params.motor_group_state_channel_names_[i]));
  }
}
RobotControlInterface::ReadParams(char* interface_config_file_,
                                  RobotControlInterface* params) {
  // Initialize the reader of the cfg file
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";

  std::cout << "Reading robot control interface parameters ..." << std::endl;
  try {
    // Parse the cfg file
    cfg->parse(config_file);

    params->num_motor_groups_ = cfg->lookupFloat(scope, "num_motor_groups");
    std::cout << "num_motor_groups: " << params->num_motor_groups_ << std::endl;

    std::stringstream ss;
    for (int i = 0; i < params->num_motor_groups_; i++) {
      // Construct strings to lookup in the cfg file
      ss1 << "motor_group" << i + 1;
      std::string motor_group_str = ss1.str();
      std::string motor_group_joints_str = motor_group_str + "_joints";
      std::string motor_group_cmd_chan_str = motor_group_str + "_cmd_chan";
      std::string motor_group_state_chan_str = motor_group_str + "_state_chan";

      // Lookup motor group name
      params->motor_group_names_.push_back(
          std::string(cfg->lookupString(scope, motor_group_str.c_str())));
      std::cout << motor_group_str << ": " << params->motor_group_names_[i];

      // Lookup motor group joint names list. The list is a space separated.
      // So we split the string by spaces and store it as a vector
      std::string motor_group_all_joints(
          cfg->lookupString(scope, motor_group_joints_str.c_str()));
      std::stringstream ss2(motor_group_all_joints);
      std::istream_iterator<std::string> begin(ss2);
      std::istream_iterator<std::string> end;
      params->motor_group_joints_.push_back(
          std::vector<std::string>(begin, end));

      params->motor_group_joints_.push_back(std::string());
      std::cout << motor_group_joints_str << ": "
                << params->motor_group_joints_[i];

      // Lookup command channel name for the motor group
      params->motor_group_command_channel_names_.push_back(std::string(
          cfg->lookupString(scope, motor_group_cmd_chan_str.c_str())));
      std::cout << motor_group_cmd_chan_str << ": "
                << params->motor_group_command_channel_names_[i];

      // Lookup state channel name for the motor group
      params->motor_group_state_channel_names_.push_back(std::string(
          cfg->lookupString(scope, motor_group_state_chan_str.c_str())));
      std::cout << motor_group_state_chan_str << ": "
                << params->motor_group_state_channel_names_[i];

      ss1.clear();
      ss2.clear();
    }
    for (int i = 0; i < params->num_sensor_groups_; i++) {
      // Construct strings to lookup in the cfg file
      ss << "sensor_group" << i + 1;
      std::string sensor_group_str = ss.str();
      std::string sensor_group_state_chan_str = sensor_group + "_state_chan";

      // Lookup sensor group name
      params->sensor_group_names_.push_back(
          std::string(cfg->lookupString(scope, sensor_group_str.c_str())));
      std::cout << sensor_group_str << ": " << params->sensor_group_joints_[i];

      // Lookup state channel name for the sensor group
      params->sensor_group_state_channel_names_.push_back(std::string(
          cfg->lookupString(scope, sensor_group_state_chan_str.c_str())));
      std::cout << sensor_group_state_chan_str << ": "
                << params->sensor_group_state_channel_names_[i];

      ss.clear();
    }
  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading robot control interface parameters");
  }
  std::cout << std::endl;
}
RobotControlInterface::Destroy() {
  for (int i = 0; i < motor_groups_.size(); i++) {
    motor_groups_[i]->Destroy();
    delete motor_groups_[i];
  }
  motor_group_.clear();
  for (int i = 0; i < sensor_groups_.size(); i++) {
    sensor_groups_[i]->Destroy();
    delete sensor_groups_[i];
  }
  sensor_groups_.clear();
  interface_context_.Destroy();
}
RobotControlInterface::Run() {
  for (int i = 0; i < motor_groups_.size(); i++) motor_groups_[i]->Run();
  for (int i = 0; i < sensor_groups_.size(); i++) sensor_groups_[i]->Run();
}
