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
 * @file sensor_group.h
 * @author Munzir Zafar
 * @date Nov 24, 2018
 * @brief Header for sensor_group.cpp that communicates sensor outputs to
 * communication channels
 */

#ifndef KRANG_SIMULATION_SENSOR_GROUP_H_
#define KRANG_SIMULATION_SENSOR_GROUP_H_

#include <dart/dart.hpp>  // dart::dynamics
#include <mutex>          // std::mutex
#include <string>         // std::string
#include <thread>         // std::thread

#include "ach_interface.h"  // InterfaceContext, SensorInterfaceBase
#include "sensor_base.h"    // SensorBase, sensor::

namespace krang_sim_ach {

class SensorGroup {
 public:
  SensorGroup(dart::dynamics::SkeletonPtr robot,
              InterfaceContext& interface_context,
              std::string& sensor_group_name,
              std::string& sensor_group_state_channel_name);

  ~SensorGroup() { Destroy(); }

  void Run();

  void InfiniteRun();

  void Destroy();

 public:
  std::mutex robot_mutex_;

 private:
  SensorBase* sensor_;
  SensorInterfaceBase* interface_;

  std::thread* thread_;
  bool run_;
  std::mutex run_mutex_;
};

} // namespace krang_sim_ach
#endif  // KRANG_SIMULATION_SENSOR_GROUP_H_
