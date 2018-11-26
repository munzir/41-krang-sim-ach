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

#include <dart/dart.hpp>  // dart::dynamics::
#include <string>         // std::string

WaistMotor::WaistMotor(dart::dynamics::SkeletonPtr robot,
                       std::string& joint_name, const char* motor_config_file) {}
void WaistMotor::Update() {}
void WaistMotor::Destroy() {}
void WaistMotor::Lock() {}
void WaistMotor::Unlock() {}
void WaistMotor::PositionCmd(double val) {}
void WaistMotor::VelocityCmd(double val) {}
void WaistMotor::CurrentCmd(double val) {}
double WaistMotor::GetPosition() {return 0.0;}
double WaistMotor::GetVelocity() {return 0.0;}
double  WaistMotor::GetCurrent() {return 0.0;}
std::string WaistMotor::GetMotorType() {return "waist";}
