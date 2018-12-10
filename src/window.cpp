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
 * @file window.cpp
 * @author Munzir Zafar
 * @date Nov 14, 2018
 * @brief Manages the window rendering the animation of the robot
 */
#include "window.h"

#include <dart/dart.hpp>  // dart::simulation::WorldPtr, dart::dynamics::SkeletonPtr
#include "ach_interface.h"            // WorldInterface::SimCmd
#include "dart_world.h"               // SetKrangInitPose()
#include "robot_control_interface.h"  // RobotControlInterface

MyWindow::MyWindow(const dart::simulation::WorldPtr& world,
                   RobotControlInterface* robot_control_interface)
    : robot_control_interface_(robot_control_interface) {
  // Attach the world passed in the input argument to the window
  setWorld(world);
}

void MyWindow::timeStepping() {
  std::cout << std::endl
            << "                                                   world "
               "receiving command";
  std::cout.flush();
  WorldInterface::SimCmd sim_cmd =
      robot_control_interface_->world_interface_->ReceiveCommand();
  if (robot_control_interface_->external_timestepping_ == false ||
      sim_cmd == WorldInterface::kStep) {
    // Lock all mutexes
    std::cout << std::endl
              << "                                                   world "
                 "locking mutex";
    std::cout.flush();
    robot_control_interface_->MutexLock();

    // Step the world through time
    std::cout << std::endl
              << "                                                   world "
                 "timestepping";
    std::cout.flush();
    SimWindow::timeStepping();

    // Unlock all mutexes
    std::cout << std::endl
              << "                                                   world "
                 "unlocking mutex";
    std::cout.flush();
    robot_control_interface_->MutexUnlock();
  }

  if (sim_cmd == WorldInterface::kReset) {
    // Lock all mutexes
    robot_control_interface_->MutexLock();

    // Set initial pose to the one commanded
    dart::dynamics::SkeletonPtr robot = mWorld->getSkeleton("krang");
    SetKrangInitPos(robot_control_interface_->world_interface_->pose_params_,
                    robot);

    // Set initial speeds to be zero
    robot->setVelocities(Eigen::VectorXd::Zero(robot->getNumDofs()));

    // Lock all joints
    for (int i = 0; i < robot_control_interface_->motor_groups_.size(); i++) {
      robot_control_interface_->motor_groups_[i]->Execute(MotorBase::kLock);
    }

    // Unlock all mutexes
    robot_control_interface_->MutexUnlock();
  }
}
