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
 * @file window.h
 * @author Munzir Zafar
 * @date Nov 14, 2018
 * @brief Header for MyWindow.cpp that manages the window rendering the
 * animation of the robot
 */

#ifndef KRANG_SIMULATION_WINDOW_H_
#define KRANG_SIMULATION_WINDOW_H_

#include <dart/dart.hpp>     //dart::simulation::WorldPtr
#include <dart/gui/gui.hpp>  // dart::gui::SimWindow
#include <fstream>

#include "robot_control_interface.h"  // RobotControlInterface

namespace krang_sim_ach {

class MyWindow : public dart::gui::glut::SimWindow {
 public:
  MyWindow(const dart::simulation::WorldPtr& world,
           RobotControlInterface* robot_control_interface,
           bool* sig_received, bool marks_on_ground);

  ~MyWindow() { out_file_.close(); }

  void timeStepping() override;
  void drawWorld() const;

 public:
  RobotControlInterface* robot_control_interface_;
  bool* sig_received_;
  std::ofstream out_file_;
  bool marks_on_ground_;
};

} // namespace krang_sim_ach
#endif  // KRANG_SIMULATION_WINDOW_H_
