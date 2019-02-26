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
 * @file waist_motor.h
 * @author Munzir Zafar
 * @date Nov 26, 2018
 * @brief Waist Motor simulation
 */

#ifndef KRANG_SIMULATION_WAIST_MOTOR_H_
#define KRANG_SIMULATION_WAIST_MOTOR_H_

#include "motor_base.h"

#include <dart/dart.hpp>  // dart::dynamics::
#include <string>         // std::string

namespace krang_sim_ach {

class WaistMotor : public MotorBase {
 public:
  WaistMotor(dart::dynamics::SkeletonPtr robot, std::string& joint_name,
              const char* motor_config_file);
  ~WaistMotor() { Destroy(); }
  void Update() override;
  void Destroy() override;
  void Lock() override;
  void Unlock() override;
  void PositionCmd(double val) override;
  void VelocityCmd(double val) override;
  void CurrentCmd(double val) override;
  double GetPosition() override;
  double GetVelocity() override;
  double GetCurrent() override;
  std::string GetMotorType() override;

  dart::dynamics::JointPtr joint_;
  double motor_power_;
  double nominal_torque_;
  double peak_torque_;
  double max_angular_velocity_;
  double max_angular_acceleration_;
  double gear_ratio_;
  double nominal_power_current_;
  double max_current_;

  double coulomb_friction_;
  double viscous_friction_;
  struct Gains {
    double Kp_;
    double Kd_;
  } position_ctrl_gains_, speed_ctrl_gains_;
  void ReadParams(const char* motor_param_file);
  enum Mode { kLocked, kPositionCtrl, kVelocityCtrl, kCurrentCtrl } mode_;
  double reference_position_;
  double reference_speed_;
  double reference_current_;
};

} // namespace krang_sim_ach
#endif  // KRANG_SIMULATION_WAIST_MOTOR_H_
