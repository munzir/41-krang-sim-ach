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
 * @file waist_motor.cpp
 * @author Munzir Zafar
 * @date Nov 26, 2018
 * @brief Waist Motor simulation
 */

#include "waist_motor.h"

#include <assert.h>
#include <config4cpp/Configuration.h>  // config4cpp::Configuration
#include <algorithm>                   // std::min, std::max
#include <dart/dart.hpp>               // dart::dynamics::
#include <iostream>                    // std::cerr
#include <sstream>                     // istringstream
#include <string>                      // std::string

WaistMotor::WaistMotor(dart::dynamics::SkeletonPtr robot,
                       std::string& joint_name, const char* motor_config_file) {
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
  // TODO: How to avoid the hardcoded path to cfg/waist folder?
  ReadParams(("/usr/local/share/krang-sim-ach/cfg/" + motor_make + "/" +
              motor_model + ".cfg")
                 .c_str());

  const double kRadiansPerDegree = M_PI / 180.0;
  joint_->setForceUpperLimit(0, peak_torque_);
  joint_->setForceLowerLimit(0, -peak_torque_);
  joint_->setVelocityUpperLimit(0, max_angular_velocity_ * kRadiansPerDegree);
  joint_->setVelocityLowerLimit(0, -max_angular_velocity_ * kRadiansPerDegree);
  joint_->setAccelerationUpperLimit(
      0, max_angular_acceleration_ * kRadiansPerDegree);
  joint_->setAccelerationLowerLimit(
      0, -max_angular_acceleration_ * kRadiansPerDegree);
  joint_->setCoulombFriction(0, coulomb_friction_);
  joint_->setDampingCoefficient(0, viscous_friction_);

  Lock();
}

void WaistMotor::ReadParams(const char* motor_param_file) {
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";
  const char* str;
  std::istringstream stream;
  std::cout << "Reading parameters from " << std::string(motor_param_file)
            << " ... " << std::endl;
  try {
    cfg->parse(motor_param_file);

    motor_power_ = cfg->lookupFloat(scope, "motor_power");
    std::cout << "motor_power: " << motor_power_ << std::endl;

    nominal_torque_ = cfg->lookupFloat(scope, "nominal_torque");
    std::cout << "nominal_torque: " << nominal_torque_ << std::endl;

    peak_torque_ = cfg->lookupFloat(scope, "peak_torque");
    std::cout << "peak_torque: " << peak_torque_ << std::endl;

    max_angular_velocity_ = cfg->lookupFloat(scope, "max_angular_velocity");
    std::cout << "max_angular_velocity: " << max_angular_velocity_ << std::endl;

    max_angular_acceleration_ =
        cfg->lookupFloat(scope, "max_angular_acceleration");
    std::cout << "max_angular_acceleration: " << max_angular_acceleration_
              << std::endl;

    gear_ratio_ = cfg->lookupFloat(scope, "gear_ratio");
    std::cout << "gear_ratio: " << gear_ratio_ << std::endl;

    nominal_power_current_ = cfg->lookupFloat(scope, "nominal_power_current");
    std::cout << "nominal_power_current: " << nominal_power_current_
              << std::endl;

    max_current_ = cfg->lookupFloat(scope, "max_current");
    std::cout << "max_current: " << max_current_ << std::endl;

    coulomb_friction_ = cfg->lookupFloat(scope, "coulomb_friction");
    std::cout << "coulomb_friction: " << coulomb_friction_ << std::endl;

    viscous_friction_ = cfg->lookupFloat(scope, "viscous_friction");
    std::cout << "viscous_friction: " << viscous_friction_ << std::endl;

    str = cfg->lookupString(scope, "position_ctrl_pd_gains");
    stream.str(str);
    stream >> position_ctrl_gains_.Kp_;
    stream >> position_ctrl_gains_.Kd_;
    stream.clear();
    std::cout << "position_ctrl_pd_gains: " << position_ctrl_gains_.Kp_ << ", "
              << position_ctrl_gains_.Kd_ << std::endl;

    str = cfg->lookupString(scope, "speed_ctrl_pd_gains");
    stream.str(str);
    stream >> speed_ctrl_gains_.Kp_;
    stream >> speed_ctrl_gains_.Kd_;
    stream.clear();
    std::cout << "speed_ctrl_pd_gains: " << speed_ctrl_gains_.Kp_ << ", "
              << speed_ctrl_gains_.Kd_ << std::endl;
  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading motor params");
  }
}
void WaistMotor::Update() {
  switch (mode_) {
    case kLocked: {
      break;
    }
    case kPositionCtrl: {
      double current_position = joint_->getPosition(0);
      double current_speed = joint_->getVelocity(0);
      double reference_speed = 0.0;
      double torque =
          -position_ctrl_gains_.Kp_ * (current_position - reference_position_) -
          position_ctrl_gains_.Kd_ * (current_speed - reference_speed);
      joint_->setForce(0,
                       std::min(std::max(torque, -peak_torque_), peak_torque_));
      break;
    }
    case kVelocityCtrl: {
      double current_speed = joint_->getVelocity(0);
      double torque =
          -speed_ctrl_gains_.Kd_ * (current_speed - reference_speed_);
      joint_->setForce(0,
                       std::min(std::max(torque, -peak_torque_), peak_torque_));
      break;
    }
    case kCurrentCtrl: {
      double torque = reference_current_ * peak_torque_ / max_current_;
      joint_->setForce(0,
                       std::min(std::max(torque, -peak_torque_), peak_torque_));
      break;
    }
  }
}
void WaistMotor::Destroy() {}

void WaistMotor::Lock() {
  mode_ = kLocked;
  joint_->setActuatorType(dart::dynamics::Joint::ActuatorType::LOCKED);
}

void WaistMotor::Unlock() {
  mode_ = kCurrentCtrl;
  reference_current_ = 0.0;
  joint_->setActuatorType(dart::dynamics::Joint::ActuatorType::FORCE);
}

void WaistMotor::PositionCmd(double val) {
  mode_ = kPositionCtrl;
  reference_position_ = val;
  joint_->setActuatorType(dart::dynamics::Joint::ActuatorType::FORCE);
}
void WaistMotor::VelocityCmd(double val) {
  mode_ = kVelocityCtrl;
  reference_speed_ = val;
  joint_->setActuatorType(dart::dynamics::Joint::ActuatorType::FORCE);
}

void WaistMotor::CurrentCmd(double val) {
  mode_ = kCurrentCtrl;
  reference_current_ = val;
  joint_->setActuatorType(dart::dynamics::Joint::ActuatorType::FORCE);
}

double WaistMotor::GetPosition() { return joint_->getPosition(0); }

double WaistMotor::GetVelocity() { return joint_->getVelocity(0); }

double WaistMotor::GetCurrent() {
  return (joint_->getForce(0) * max_current_ / peak_torque_);
}

std::string WaistMotor::GetMotorType() { return "waist"; }
