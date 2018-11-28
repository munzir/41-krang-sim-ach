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
 * @file krang-sim-ach.cpp
 * @author Munzir Zafar
 * @date Nov 14, 2018
 * @brief Executable that runs krang simulation and allows other programs
 * to interact with the simulation via ach channels
 */
#include <stdlib.h>  // atexit()

#include <dart/dart.hpp>  // dart::simulation::WorldPtr, glutInit(), glutMainLoop()

#include "dart_world.h"               // CreateWorld()
#include "robot_control_interface.h"  // RobotControlInterface
#include "window.h"                   // MyWindow

//=============================================================================
// This function is written because glutMainLoop() does not ever return.
// Instead it calls exit(0). The cleaning that could be elegantly done in
// RobotControlInterface destructor has now to be done explicitly, because no
// destructor will be executed upon exit. By making use of atexit(ExitFunction),
// we will force the execution of ExitFunction() upon exit. Since this function
// is not allowed to accept any arguments, we are defining RobotControlInterface
// pointer as a global variable to allow this function to access the
// RobotControlInterface object that is to be destroyed.
RobotControlInterface* krang_ach;
void ExitFunction() { krang_ach->Destroy(); }

//=============================================================================
int main(int argc, char* argv[]) {
  // Creates world and loads all objects with desired initial configuration
  char path_to_dart_params[] =
      "/home/munzir/Me/5-Work/01-PhD/01-WholeBodyControlAttempt1/"
      "41-krang-sim-ach/cfg/dart_params.cfg";
  dart::simulation::WorldPtr world = CreateWorld(path_to_dart_params);

  // Create interface that allows other programs to interface with out robot
  char path_to_motor_params[] =
      "/home/munzir/Me/5-Work/01-PhD/01-WholeBodyControlAttempt1/"
      "41-krang-sim-ach/cfg/krang_motors.cfg";
  char path_to_interface_params[] =
      "/home/munzir/Me/5-Work/01-PhD/01-WholeBodyControlAttempt1/"
      "41-krang-sim-ach/cfg/ach_params.cfg";
  krang_ach =
      new RobotControlInterface(world->getSkeleton("krang"),
                                path_to_motor_params, path_to_interface_params);

  // The window object that has all the callback functions defined which are
  // executed during simulation. It therefore needs access to all other objects
  // in the program. Hence, it is being provided with them all at construction
  MyWindow window(world, krang_ach);

  // Define exit function
  atexit(ExitFunction);

  // Glut init
  glutInit(&argc, argv);

  // Launch the window. Specify glut callback functions as methods of the window
  // object
  window.initWindow(1280, 720, "Krang Simulation with Ach");

  // Process events of the window rendering our simulation. This thread never
  // returns. Calls exit(0) when the window is closed.
  glutMainLoop();

  return 0;
}
