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
 * @file amc_motor.cpp
 * @author Munzir Zafar
 * @date Nov 26, 2018
 * @brief Amc Motor simulation
 */

#include "amc_motor.h"

#include <assert.h>
#include <config4cpp/Configuration.h>  // config4cpp::Configuration
#include <dart/dart.hpp>               // dart::dynamics::
#include <iostream>                    // std::cerr
#include <sstream>                     // istringstream
#include <string>                      // std::string

AmcMotor::AmcMotor(dart::dynamics::SkeletonPtr robot, std::string& joint_name,
                   const char* motor_config_file) {
  joint_ = robot->getJoint(joint_name);

  // Get the motor model from the motor_config_file
  std::string motor_make, motor_model;
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";
  try {
    cfg->parse(motor_config_file);
    motor_make =
        std::string(cfg->lookupString(scope, (joint_name + "_make").c_str()));
    motor_model =
        std::string(cfg->lookupString(scope, (joint_name + "_model").c_str()));
  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading motor model from motor config file");
  }

  // Read parameters from file
  // TODO: How to avoid the hardcoded path to cfg/amc folder?
  ReadParams(("../cfg/" + motor_make + "/" + motor_model + ".cfg").c_str());

  const double Newton_meters_per_pound_inch = 0.11298482933333;
  const double radians_per_second_per_rpm = 0.104719755;
  joint_->setForceUpperLimit(0,
                             max_output_torque_ * Newton_meters_per_pound_inch);
  joint_->setForceLowerLimit(
      0, -max_output_torque_ * Newton_meters_per_pound_inch);
  joint_->setVelocityUpperLimit(
      0, 1.5 * rated_speed_ * radians_per_second_per_rpm);
  joint_->setVelocityLowerLimit(
      0, -1.5 * rated_speed_ * radians_per_second_per_rpm);
  joint_->setDampingCoefficient(0, viscous_friction_);

  CurrentCmd(0.0);
}

void AmcMotor::ReadParams(const char* motor_param_file) {
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";
  const char* str;
  std::istringstream stream;
  std::cout << "Reading parameters from " << std::string(motor_param_file)
            << " ... " << std::endl;
  try {
    cfg->parse(motor_param_file);

    rated_voltage_ = cfg->lookupFloat(scope, "rated_voltage");
    std::cout << "rated_voltage: " << rated_voltage_ << std::endl;

    rated_speed_ = cfg->lookupFloat(scope, "rated_speed");
    std::cout << "rated_speed: " << rated_speed_ << std::endl;

    rated_torque_ = cfg->lookupFloat(scope, "rated_torque");
    std::cout << "rated_torque: " << rated_torque_ << std::endl;

    rated_power_ = cfg->lookupFloat(scope, "rated_power");
    std::cout << "rated_power: " << rated_power_ << std::endl;

    rated_current_ = cfg->lookupFloat(scope, "rated_current");
    std::cout << "rated_current: " << rated_current_ << std::endl;

    line_to_line_resistance_ =
        cfg->lookupFloat(scope, "line_to_line_resistance");
    std::cout << "line_to_line_resistance: " << line_to_line_resistance_
              << std::endl;

    line_to_line_inductance_ =
        cfg->lookupFloat(scope, "line_to_line_inductance");
    std::cout << "line_to_line_inductance: " << line_to_line_inductance_
              << std::endl;

    torque_constant_ = cfg->lookupFloat(scope, "torque_constant");
    std::cout << "torque_constant: " << torque_constant_ << std::endl;

    back_emf_voltage_ = cfg->lookupFloat(scope, "back_emf_voltage");
    std::cout << "back_emf_voltage: " << back_emf_voltage_ << std::endl;

    rotor_inertia_ = cfg->lookupFloat(scope, "rotor_inertia");
    std::cout << "rotor_inertia: " << rotor_inertia_ << std::endl;

    gear_ratio_ = cfg->lookupFloat(scope, "gear_ratio");
    std::cout << "gear_ratio: " << gear_ratio_ << std::endl;

    rated_output_torque_ = cfg->lookupFloat(scope, "rated_output_torque");
    std::cout << "rated_output_torque: " << rated_output_torque_ << std::endl;

    stages_ = cfg->lookupFloat(scope, "stages");
    std::cout << "stages: " << stages_ << std::endl;

    max_backlash_ = cfg->lookupFloat(scope, "max_backlash");
    std::cout << "max_backlash: " << max_backlash_ << std::endl;

    viscous_friction_ = cfg->lookupFloat(scope, "viscous_friction");
    std::cout << "viscous_friction: " << viscous_friction_ << std::endl;

  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading motor params");
  }
}
void AmcMotor::Update() {
  const double Newton_meters_per_ounce_inch = 0.00706155183333;
  double torque = reference_current_ *
                  (Newton_meters_per_ounce_inch * torque_constant_) *
                  gear_ratio_;
  joint_->setForce(0, torque);
}
void AmcMotor::Destroy() {}

void AmcMotor::Lock() {}

void AmcMotor::Unlock() {}

void AmcMotor::PositionCmd(double val) {}
void AmcMotor::VelocityCmd(double val) {}

void AmcMotor::CurrentCmd(double val) {
  reference_current_ = val;
  joint_->setActuatorType(dart::dynamics::Joint::ActuatorType::FORCE);
}

double AmcMotor::GetPosition() { return joint_->getPosition(0); }

double AmcMotor::GetVelocity() { return joint_->getVelocity(0); }

double AmcMotor::GetCurrent() {
  const double Newton_meters_per_ounce_inch = 0.00706155183333;
  return (joint_->getForce(0) /
          (Newton_meters_per_ounce_inch * torque_constant_) / gear_ratio_);
}

std::string AmcMotor::GetMotorType() { return "amc"; }
