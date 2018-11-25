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
 * @file ach_interface.h
 * @author Munzir Zafar
 * @date Nov 25, 2018
 * @brief Provides ach interface to robot
 */

#ifndef KRANG_SIMULATION_ACH_INTERFACE_H_
#define KRANG_SIMULATION_ACH_INTERFACE_H_

#include "motor_base.h"   // MotorBase::MotorCommandType, MotorBase*
#include "sensor_base.h"  // SensorBase*

#include <amino.h>           // needed by ach.h
#include <somatic.pb-c.h>    // Somatic__Vector, Somatic__MotorState
#include <somatic/daemon.h>  // somatic_d_t, somatic_d_opts_t

#include <ach.h>  // ach_channel_t

#include <string>  // std::string
#include <vector>  // std::vector

class InterfaceContext {
 public:
  void Init(const char* interface_config_file);
  void Destroy();
  struct InterfaceContextParams {
    char daemon_identifier_[128];
    bool daemonize_;
  };
  void ReadParams(char* interface_config_file, InterfaceContextParams* params);
  somatic_d_t daemon_;
  somatic_d_opts_t daemon_opts_;
}

class SensorInterfaceBase {
 public:
  virtual void SendState();
  virtual void Destroy();
}

class MotorInterfaceBase {
 public:
  virtual void ReceiveCommand(MotorBase::MotorCommandType* command,
                              std::vector<double>* command_val_);
  virtual void SendState();
  virtual void Destroy();
}

class FloatingBaseStateSensorInterface : public SensorInterfaceBase {
 public:
  FloatingBaseStateSensorInterface(SensorBase* sensor,
                                   InterfaceContext& interface_context,
                                   std::string& sensor_state_channel);
  ~FloatingBaseStateSensorInterface() { Destroy(); }
  void SendState() override;
  void Destroy() override;

  const SensorBase* sensor_;  // const because interface should only be able to
                              // read the sensor, not modify it
  ach_channel_t imu_chan_;
  Somatic__Vector* imu_msg_;
  somatic_d_t* daemon_;
}

class SchunkMotorInterface : public MotorInterfaceBase {
 public:
  SchunkMotorInterface(std::vector<MotorBase*>& motor_vector,
                       InterfaceContext& interface_context,
                       std::string& motor_group_name,
                       std::string& motor_group_command_channel_name,
                       std::string& motor_group_state_channel_name);
  ~SchunkMotorInterface() { Destroy(); }
  void ReceiveCommand(MotorBase::MotorCommandType* command,
                      std::vector<double>* command_val_) override;
  void SendState() override;
  void Destroy() override;
  void InitMessage();

  const std::vector<MotorBase*>*
      motor_vector_ptr_;  // const because interface should only be able to read
                          // from the motor
  char name_[128];
  somatic_d_t* daemon_;
  size_t n_;  // module count
  ach_channel_t cmd_chan_;
  ach_channel_t state_chan_;
  Somatic__MotorState state_msg_;
  struct {
    Somatic__Vector position_;
    Somatic__Vector velocity_;
    Somatic__Vector current_;
  } state_msg_fields_;
}

class AmcMotorInterface : public MotorInterfaceBase {
 public:
  AmcMotorInterface(std::vector<MotorBase*>& motor_vector,
                    InterfaceContext& interface_context,
                    std::string& motor_group_name,
                    std::string& motor_group_command_channel_name,
                    std::string& motor_group_state_channel_name);
  ~AmcMotorInterface() { Destroy(); }
  void ReceiveCommand(MotorBase::MotorCommandType* command,
                      std::vector<double>* command_val_) override;
  void SendState() override;
  void Destroy() override;
}

class WaistMotorInterface : public MotorInterfaceBase {
 public:
  WaistMotorInterface(std::vector<MotorBase*>& motor_vector,
                      InterfaceContext& interface_context,
                      std::string& motor_group_name,
                      std::string& motor_group_command_channel_name,
                      std::string& motor_group_state_channel_name);
  ~WaistMotorInterface() { Destroy(); }
  void ReceiveCommand(MotorBase::MotorCommandType* command,
                      std::vector<double>* command_val_) override;
  void SendState() override;
  void Destroy() override;
}

namespace interface {
  SensorInterfaceBase* Create(
      SensorBase * sensor, InterfaceContext & interface_context,
      std::string & sensor_name, std::string & sensor_state_channel);

  MotorInterfaceBase* Create(std::vector<MotorBase*> & motor_vector,
                             InterfaceContext & interface_context,
                             std::string & motor_group_name,
                             std::string & motor_group_command_channel_name,
                             std::string & motor_group_state_channel_name);
}
#endif  // KRANG_SIMULATION_ACH_INTERFACE_H_
