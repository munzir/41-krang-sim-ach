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
 * @file sensor_group.cpp
 * @author Munzir Zafar
 * @date Nov 26, 2018
 * @brief Communicates sensor outputs to communication channels
 */

#include "sensor_group.h"

#include <dart/dart.hpp>  // dart::dynamics
#include <string>         // std::string
#include <thread>         // std::thread

#include "ach_interface.h"  // InterfaceContext, SensorInterfaceBase
#include "sensor_base.h"    // SensorBase, sensor::

SensorGroup::SensorGroup(dart::dynamics::SkeletonPtr robot,
                         InterfaceContext& interface_context,
                         std::string& sensor_group_name,
                         std::string& sensor_group_state_channel_name) {
  sensor_ = sensor::Create(robot, sensor_group_name);
  interface_ = interface::Create(sensor_, interface_context, sensor_group_name,
                                 sensor_group_state_channel_name);
  thread_ = new std::thread(&SensorGroup::InfiniteRun, this);
}

void SensorGroup::Run() {
  sensor_->Update();
  interface_->SendState();
}

void SensorGroup::Destroy() {
  delete thread_;
  sensor_->Destroy();
  interface_->Destroy();
}
