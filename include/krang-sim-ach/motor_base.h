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
 * @file motor_base.h
 * @author Munzir Zafar
 * @date Nov 24, 2018
 * @brief Base class for all motors
 */

#ifndef KRANG_SIMULATION_MOTOR_BASE_H_
#define KRANG_SIMULATION_MOTOR_BASE_H_

#include <dart/dart.hpp>  // dart::dynamics::
#include <string>         // std::string
#include <vector>         // std::vector

namespace krang_sim_ach {

class MotorBase {
 public:
  enum MotorCommandType {
    kLock = 0,
    kPosition,
    kVelocity,
    kCurrent,
    kUnlock,
    kDoNothing
  };
  MotorBase() {}
  ~MotorBase() {}
  virtual void Update() = 0;
  virtual void Destroy() = 0;
  virtual void Lock() = 0;
  virtual void Unlock() = 0;
  virtual void PositionCmd(double val) = 0;
  virtual void VelocityCmd(double val) = 0;
  virtual void CurrentCmd(double val) = 0;
  virtual double GetPosition() = 0;
  virtual double GetVelocity() = 0;
  virtual double GetCurrent() = 0;
  virtual std::string GetMotorType() = 0;
};

namespace motor {
MotorBase* Create(dart::dynamics::SkeletonPtr robot, std::string& joint_name,
                  const char* motor_config_file);
}

} // namespace krang_sim_ach

#endif  // KRANG_SIMULATION_MOTOR_BASE_H_
