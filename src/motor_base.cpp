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
 * @file motor_base.cpp
 * @author Munzir Zafar
 * @date Nov 15, 2018
 * @brief Base class for all motors
 */

#include "motor_base.h"

#include "amc_motor.h"     // AmcMotor
#include "schunk_motor.h"  // SchunkMotor
#include "waist_motor.h"   // WaistMotor

#include <assert.h>                    // assert()
#include <config4cpp/Configuration.h>  // config4cpp::
#include <dart/dart.hpp>               // dart::dynamics::
#include <iostream>                    // std::cerr
#include <string>                      // std::string
#include <vector>                      // std::vector

namespace krang_sim_ach {

MotorBase* motor::Create(dart::dynamics::SkeletonPtr robot,
                         std::string& joint_name,
                         const char* motor_config_file) {
  // Get the motor brand name (or make) from the motor_config_file
  std::string motor_name;
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";
  try {
    cfg->parse(motor_config_file);
    motor_name =
        std::string(cfg->lookupString(scope, (joint_name + "_make").c_str()));
  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading motor config file");
  }

  // Based on the motor brand name call the relevant constructor
  if (!motor_name.compare("schunk")) {
    return new SchunkMotor(robot, joint_name, motor_config_file);
  } else if (!motor_name.compare("amc")) {
    return new AmcMotor(robot, joint_name, motor_config_file);
  } else if (!motor_name.compare("waist")) {
    return new WaistMotor(robot, joint_name, motor_config_file);
  } else {
    assert(false && "Motor name not listed");
  }
}

} // namespace krang_sim_ach
