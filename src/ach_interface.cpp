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
 * @file ach_interface.cpp
 * @author Munzir Zafar
 * @date Nov 24, 2018
 * @brief Ach communication interface for the robot
 */

#include "ach_interface.h"

#include "motor_base.h"   // MotorBase*, MotorBase::MotorCommandType
#include "sensor_base.h"  // SensorBase*

#include <amino.h>                     // for ach.h to compile
#include <config4cpp/Configuration.h>  // config4cpp::Configuration
#include <somatic.h>                   // SOMATIC_PACK_SEND
#include <somatic.pb-c.h>  // SOMATIC__: EVENT, MSG_TYPE; somatic__anything__init()
#include <somatic/daemon.h>  // somatic_d: _init(), _event(), _channel_open(), _check()
#include <somatic/msg.h>  // somatic_anything_alloc(), somatic_anything_free()
#include <stdio.h>        // std::cout
#include <string.h>       // strdup()
#include <time.h>         // clock_gettime()
#include <cstring>        // std::memset, strcpy
#include <string>         // std::string
#include <vector>         // std::vector

#include <ach.h>  // ach_status_t, ach_result_to_string, ach_flush(), ach_close()

void InterfaceContext::Init(char* interface_config_file) {
  // Read the config file
  InterfaceContextParams params;
  ReadParams(interface_config_file, &params);

  // Initialize the daemon_
  std::memset(&daemon_opts_, 0, sizeof(daemon_opts_));
  daemon_opts_.ident = strdup(params.daemon_identifier_);
  daemon_opts_.daemonize = params.daemonize_;
  daemon_opts_.sched_rt = SOMATIC_D_SCHED_MOTOR;  // TODO: Is this needed?
  somatic_verbprintf_prefix = daemon_opts_.ident;
  std::memset(&daemon_, 0, sizeof(daemon_));
  somatic_d_init(&daemon_, &daemon_opts_);
  frequency_ = params.frequency_;

  // Send a "running" notice on the event channel
  somatic_d_event(&daemon_, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

void InterfaceContext::ReadParams(char* interface_config_file,
                                  InterfaceContextParams* params) {
  // Initialize the reader of the cfg file
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";

  std::cout << "Reading interface context parameters ..." << std::endl;
  try {
    // Parse the cfg file
    cfg->parse(interface_config_file);

    // Daemon identifier
    params->daemon_identifieri_ = cfg->lookupString(scope, "daemon_identifier");
    std::cout << "daemon identifier: " << params->daemon_identifier_
              << std::endl;

    // Daemonize?
    params->daemonize_ = lookupBoolean(scope, "daemonize");
    std::cout << "daemonize: " << (params->daemonize_ ? "true" : "false")
              << std::endl;

    // Refresh frequency (Hz)
    params->frequency_ = lookupFloat(scope, "frequency");
    std::cout << "frequency: " << params->frequency_ << std::endl;

  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading interface context parameters");
  }
  std::cout << std::endl;
}

void InterfaceContext::Destroy() {
  // Send a "stopping" notice on the event channel
  somatic_d_event(&daemon_, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

  // Destroy the daemon
  somatic_d_destroy(&daemon_);
}

FloatingBaseStateSensorInterface::FloatingBaseStateSensorInterface(
    SensorBase* sensor, InterfaceContext& interface_context,
    std::string& sensor_state_channel)
    : sensor_(sensor) {
  daemon_ = &interface_context.daemon_;

  // Open ach channel for imu data communication, and allocate memory for the
  // message
  somatic_d_channel_open(daemon_, &imu_chan_, sensor_state_channel.c_str(),
                         NULL);
  imu_msg_ = somatic_vector_alloc(6);
}

void FloatingBaseStateSensorInterface::SendState() override {
  imu_msg_->data[0] = sensor_->gravity_direction_.x_;
  imu_msg_->data[1] = sensor_->gravity_direction_.y_;
  imu_msg_->data[2] = sensor_->gravity_direction_.z_;
  imu_msg_->data[3] = sensor_->angular_velocity_.x_;
  imu_msg_->data[4] = sensor_->angular_velocity_.y_;
  imu_msg_->data[5] = sensor_->angular_velocity_.z_;

  ach_status_t r = SOMATIC_PACK_SEND(&imu_chan_, somatic__vector, imu_msg_);
  somatic_d_check(daemon_, SOMATIC__EVENT__PRIORITIES__CRIT,
                  SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT, ACH_OK == r,
                  "SendState", "imu, ach result: %s", ach_result_to_string(r));
}

void FloatingBaseStateSensorInterface::Destroy() override {
  // Clean up imu stuff
  ach_close(&imu_chan_);
  free(imu_msg_->data);
  free(imu_msg_);
}

SchunkMotorInterface::SchunkMotorInterface(
    std::vector<MotorBase*>& motor_vector, InterfaceContext& interface_context,
    std::string& motor_group_name,
    std::string& motor_group_command_channel_name,
    std::string& motor_group_state_channel_name) {
  motor_vector_ptr_ = &motor_vector;
  daemon_ = &interface_context.daemon_;
  wait_time_ = (struct timespec){
      .tv_sec = 0,
      .tv_nsec = (long int)((1.0 / interface_context.frequency_) * 1e9)};

  strcpy(name_, motor_group_name.c_str());
  n_ = motor_vector.size();

  // Open ach channels for pub/sub
  somatic_d_channel_open(daemon_, &cmd_chan_,
                         motor_group_command_channel_name.c_str(), NULL);
  somatic_d_channel_open(daemon_, &state_chan_,
                         motor_group_state_channel_name.c_str(), NULL);
  ach_flush(&cmd_chan_);

  // Assign default values to message
  InitMessage();
}

//============================================================================
/// Sets up the message we will be sending to the motor group
void MotorGroup::InitMessage() {
  // Initialize the message and set status
  somatic__motor_state__init(&state_msg_);
  state_msg_.has_status = 1;
  state_msg_.status = SOMATIC__MOTOR_STATUS__MOTOR_OK;

  // Set the addresses
  state_msg_.position = &state_msg_fields_.position_;
  state_msg_.velocity = &state_msg_fields_.velocity_;
  state_msg_.current = &state_msg_fields_.current_;

  // Initialize each of the field vectors
  somatic__vector__init(state_msg_.position);
  somatic__vector__init(state_msg_.velocity);
  somatic__vector__init(state_msg_.current);

  // Set the number of variables
  state_msg_.position->n_data = n_;
  state_msg_.velocity->n_data = n_;
  state_msg_.current->n_data = n_;

  // Set the message type
  state_msg_.meta = somatic_metadata_alloc();
  state_msg_.meta->type = SOMATIC__MSG_TYPE__MOTOR_STATE;
  state_msg_.meta->has_type = 1;
}
void SchunkMotorInterface::ReceiveCommand(
    MotorBase::MotorCommandType* command,
    std::vector<double>* command_val) override {
  // The absolute time when receive should give up waiting
  struct timespec currTime;
  clock_gettime(CLOCK_MONOTONIC, &currTime);
  struct timespec abstime = aa_tm_add(wait_time_, currTime);

  /// Read current state from state channel
  ach_status_t r;
  Somatic__MotorCmd* cmd =
      SOMATIC_D_GET(&r, somatic__motor_cmd, daemon_, &cmd_chan_, &abstime,
                    ACH_O_WAIT | ACH_O_LAST);

  // Check message reception
  somatic_d_check(
      daemon_, SOMATIC__EVENT__PRIORITIES__CRIT,
      SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT,
      ((ACH_OK == r || ACH_MISSED_FRAME == r) && cmd) || ACH_TIMEOUT == r,
      "ReceiveCommand", "motor group: %s, ach result: %s", name_,
      ach_result_to_string(r));

  // If the message has timed out, do nothing
  if (r == ACH_TIMEOUT) {
    *command = MotorBase::kDoNothing;
    return;
  }

  // Validate the message
  else if ((ACH_OK == r || ACH_MISSED_FRAME == r) && cmd) {
    // Check if the message has one of the expected parameters
    int goodParam =
        cmd->has_param && (SOMATIC__MOTOR_PARAM__MOTOR_POSITION == cmd->param ||
                           SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY == cmd->param ||
                           SOMATIC__MOTOR_PARAM__MOTOR_CURRENT == cmd->param ||
                           SOMATIC__MOTOR_PARAM__MOTOR_HALT == cmd->param ||
                           SOMATIC__MOTOR_PARAM__MOTOR_RESET == cmd->param);

    // Check if the command has the right number of parameters if pos, vel or
    // current
    int goodValues = ((cmd->values && cmd->values->n_data == cx->n) ||
                      SOMATIC__MOTOR_PARAM__MOTOR_HALT == cmd->param ||
                      SOMATIC__MOTOR_PARAM__MOTOR_RESET == cmd->param);

    // Use somatic interface to combine the finalize the checks in case there is
    // an error
    int somGoodParam = somatic_d_check_msg(
        daemon_, goodParam, "motor_cmd",
        "invalid motor param, set: %d, val: %d", cmd->has_param, cmd->param);
    int somGoodValues = somatic_d_check_msg(daemon_, goodValues, "motor_cmd",
                                            "wrong motor count: %d, wanted %d",
                                            cmd->values->n_data, n_);

    // If both good parameter and values, execute the command; otherwise just
    // update the state
    if (somGoodParam && somGoodValues) {
      switch (cmd->param) {
        case SOMATIC__MOTOR_PARAM__MOTOR_POSITION: {
          *command = MotorBase::kPosition;
          for (int i = 0; i < n_; i++) (*command_val)[i] = cmd->values->data[i];
          break;
        }
        case SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY: {
          *command = MotorBase::kVelocity;
          for (int i = 0; i < n_; i++) (*command_val)[i] = cmd->values->data[i];
          break;
        }
        case SOMATIC__MOTOR_PARAM__MOTOR_CURRENT: {
          *command = MotorBase::kCurrent;
          for (int i = 0; i < n_; i++) (*command_val)[i] = cmd->values->data[i];
          break;
        }
        case SOMATIC__MOTOR_PARAM__MOTOR_HALT: {
          *command = MotorBase::kLock;
          break;
        }
        case SOMATIC__MOTOR_PARAM__MOTOR_RESET: {
          *command = MotorBase::kUnlock;
          break;
        }
      }
    } else {
      *command = MotorBase::kDoNothing;
    }
  }

  void SchunkMotorInterface::SendState() override {
    // Read values of our group specified by joint_indices_ in the state_msg_
    double pos_vals[n_], vel_vals[n_], cur_vals[n_];
    for (int i = 0; i < n_; i++) {
      pos_vals[i] = (*motor_vector_ptr_)[i]->GetPosition();
      vel_vals[i] = (*motor_vector_ptr_)[i]->GetVelocity();
      cur_vals[i] = (*motor_vector_ptr_)[i]->GetCurrent();
    }
    state_msg_.position->data = pos_vals;
    state_msg_.velocity->data = vel_vals;
    state_msg_.current->data = cur_vals;

    // Status message (always good)
    state_msg_.has_status = 1;
    state_msg_.status = SOMATIC__MOTOR_STATUS__MOTOR_OK;

    // Package a state message for the ack returned, and send to state channel
    ach_status_t r =
        SOMATIC_PACK_SEND(&state_chan_, somatic__motor_state, &state_msg_);

    /// Check message transmission
    somatic_d_check(daemon_, SOMATIC__EVENT__PRIORITIES__CRIT,
                    SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT, ACH_OK == r,
                    "SendState", "motor group: %s, ach result: %s", name_,
                    ach_result_to_string(r));
  }

  void SchunkMotorInterface::Destroy() override {
    std::cout << "destroy " << name_ << std::endl;
    ach_close(&cmd_chan_);
    ach_close(&state_chan_);
    somatic_metadata_free(state_msg_.meta);
  }