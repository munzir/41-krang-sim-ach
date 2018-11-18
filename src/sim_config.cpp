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
 * @file sim_config.cpp
 * @author Munzir Zafar
 * @date Nov 14, 2018
 * @brief Reads configuration parameters from a cfg file
 */

#include "sim_config.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

#include <config4cpp/Configuration.h>
#include <Eigen/Eigen>

// Function for reading configuration parameters. First argument is the location
// of cfg file from the parameters are to be read. Second argument is the output
// where the parameters are stored
void ReadConfigParams(const char* config_file, SimConfig* params) {
  // Initialize the reader of the cfg file
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";

  // Temporaries we use for reading from config4cpp structure
  const char* str;
  std::istringstream stream;

  std::cout << "Reading configuration parameters ..." << std::endl;
  try {
    // Parse the cfg file
    cfg->parse(config_file);

    // Read the path to Krang urdf file
    strcpy(params->krang_urdf_path,
           cfg->lookupString(scope, "krang_urdf_path"));
    std::cout << "krang_urdf_path: " << params->krang_urdf_path << std::endl;

    // Initial heading
    params->heading_init = cfg->lookupFloat(scope, "heading_init");
    std::cout << "heading_init: " << params->heading_init << std::endl;

    // Initial base pitch angle
    params->q_base_init = cfg->lookupFloat(scope, "q_base_init");
    std::cout << "q_base_init: " << params->q_base_init << std::endl;

    // Init base xyz location in the world frame
    str = cfg->lookupString(scope, "xyz_init");
    stream.str(str);
    for (int i = 0; i < 3; i++) stream >> params->xyz_init(i);
    stream.clear();
    std::cout << "xyz_init: " << params->xyz_init.transpose() << std::endl;

    // Initial left wheel angle
    params->q_lwheel_init = cfg->lookupFloat(scope, "q_lwheel_init");
    std::cout << "q_lwheel_init: " << params->q_lwheel_init << std::endl;

    // Initial right wheel angle
    params->q_rwheel_init = cfg->lookupFloat(scope, "q_rwheel_init");
    std::cout << "q_rwheel_init: " << params->q_rwheel_init << std::endl;

    // Initial waist angle
    params->q_waist_init = cfg->lookupFloat(scope, "q_waist_init");
    std::cout << "q_waist_init: " << params->q_waist_init << std::endl;

    // Initial torso angle
    params->q_torso_init = cfg->lookupFloat(scope, "q_torso_init");
    std::cout << "q_torso_init: " << params->q_torso_init << std::endl;

    // Initial kinect angle
    params->q_kinect_init = cfg->lookupFloat(scope, "q_kinect_init");
    std::cout << "q_kinect_init: " << params->q_kinect_init << std::endl;

    // Initial configuration of the left arm
    str = cfg->lookupString(scope, "q_left_arm_init");
    stream.str(str);
    for (int i = 0; i < 7; i++) stream >> params->q_left_arm_init(i);
    stream.clear();
    std::cout << "q_left_arm_init: " << params->q_left_arm_init.transpose()
              << std::endl;

    // Initial configuration of the right arm
    str = cfg->lookupString(scope, "q_right_arm_init");
    stream.str(str);
    for (int i = 0; i < 7; i++) stream >> params->q_right_arm_init(i);
    stream.clear();
    std::cout << "q_right_arm_init: " << params->q_right_arm_init.transpose()
              << std::endl;

    // To have initial pose as a balanced pose or not
    params->init_with_balance_pose =
        cfg->lookupBoolean(scope, "init_with_balance_pose");
    std::cout << "init_with_balance_pose: "
              << (params->init_with_balance_pose ? "true" : "false")
              << std::endl;

    // Somatic daemon identifier
    strcpy(params->daemon_identifier,
           cfg->lookupString(scope, "daemon_identifier"));
    std::cout << "daemon_identifier: " << params->daemon_identifier
              << std::endl;

    // Demonize?
    params->daemonize = cfg->lookupBoolean(scope, "daemonize");
    std::cout << "daemonize: " << (params->daemonize ? "true" : "false")
              << std::endl;

    // Channels
    strcpy(params->wheels_cmd_chan,
           cfg->lookupString(scope, "wheels_cmd_chan"));
    std::cout << "wheels_cmd_chan: " << params->wheels_cmd_chan << std::endl;

    strcpy(params->wheels_state_chan,
           cfg->lookupString(scope, "wheels_state_chan"));
    std::cout << "wheels_state_chan: " << params->wheels_state_chan
              << std::endl;

    strcpy(params->waist_cmd_chan, cfg->lookupString(scope, "waist_cmd_chan"));
    std::cout << "waist_cmd_chan: " << params->waist_cmd_chan << std::endl;

    strcpy(params->waist_state_chan,
           cfg->lookupString(scope, "waist_state_chan"));
    std::cout << "waist_state_chan: " << params->waist_state_chan << std::endl;

    strcpy(params->torso_cmd_chan, cfg->lookupString(scope, "torso_cmd_chan"));
    std::cout << "torso_cmd_chan: " << params->torso_cmd_chan << std::endl;

    strcpy(params->torso_state_chan,
           cfg->lookupString(scope, "torso_state_chan"));
    std::cout << "torso_state_chan: " << params->torso_state_chan << std::endl;

    strcpy(params->left_arm_cmd_chan,
           cfg->lookupString(scope, "left_arm_cmd_chan"));
    std::cout << "left_arm_cmd_chan: " << params->left_arm_cmd_chan << std::endl;

    strcpy(params->left_arm_state_chan,
           cfg->lookupString(scope, "left_arm_state_chan"));
    std::cout << "left_arm_state_chan: " << params->left_arm_state_chan
              << std::endl;

    strcpy(params->right_arm_cmd_chan,
           cfg->lookupString(scope, "right_arm_cmd_chan"));
    std::cout << "right_arm_cmd_chan: " << params->right_arm_cmd_chan
              << std::endl;

    strcpy(params->right_arm_state_chan,
           cfg->lookupString(scope, "right_arm_state_chan"));
    std::cout << "right_arm_state_chan: " << params->right_arm_state_chan
              << std::endl;
  }

  catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading config parameters");
  }
  std::cout << std::endl;
}

