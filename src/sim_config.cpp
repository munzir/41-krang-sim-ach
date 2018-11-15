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
    strcpy(params->krangUrdfPath, cfg->lookupString(scope, "krangUrdfPath"));
    std::cout << "krangUrdfPath: " << params->krangUrdfPath << std::endl;

    // Initial heading
    params->headingInit = cfg->lookupFloat(scope, "headingInit");
    std::cout << "headingInit: " << params->headingInit << std::endl;

    // Initial base pitch angle
    params->qBaseInit = cfg->lookupFloat(scope, "qBaseInit");
    std::cout << "qBaseInit: " << params->qBaseInit << std::endl;

    // Init base xyz location in the world frame
    str = cfg->lookupString(scope, "xyzInit");
    stream.str(str);
    for (int i = 0; i < 3; i++) stream >> params->xyzInit(i);
    stream.clear();
    std::cout << "xyzInit: " << params->xyzInit.transpose() << std::endl;

    // Initial left wheel angle
    params->qLWheelInit = cfg->lookupFloat(scope, "qLWheelInit");
    std::cout << "qLWheelInit: " << params->qLWheelInit << std::endl;

    // Initial right wheel angle
    params->qRWheelInit = cfg->lookupFloat(scope, "qRWheelInit");
    std::cout << "qRWheelInit: " << params->qRWheelInit << std::endl;

    // Initial waist angle
    params->qWaistInit = cfg->lookupFloat(scope, "qWaistInit");
    std::cout << "qWaistInit: " << params->qWaistInit << std::endl;

    // Initial torso angle
    params->qTorsoInit = cfg->lookupFloat(scope, "qTorsoInit");
    std::cout << "qTorsoInit: " << params->qTorsoInit << std::endl;

    // Initial kinect angle
    params->qKinectInit = cfg->lookupFloat(scope, "qKinectInit");
    std::cout << "qKinectInit: " << params->qKinectInit << std::endl;

    // Initial configuration of the left arm
    str = cfg->lookupString(scope, "qLeftArmInit");
    stream.str(str);
    for (int i = 0; i < 7; i++) stream >> params->qLeftArmInit(i);
    stream.clear();
    std::cout << "qLeftArmInit: " << params->qLeftArmInit.transpose()
              << std::endl;

    // Initial configuration of the right arm
    str = cfg->lookupString(scope, "qRightArmInit");
    stream.str(str);
    for (int i = 0; i < 7; i++) stream >> params->qRightArmInit(i);
    stream.clear();
    std::cout << "qRightArmInit: " << params->qRightArmInit.transpose()
              << std::endl;

    // To have initial pose as a balanced pose or not
    params->initWithBalancePose = cfg->lookupBoolean(scope, "initWithBalancePose");
    std::cout << "initWithBalancePose: "
              << (params->initWithBalancePose ? "true" : "false") << std::endl;
  }

  catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
    assert(false && "Problem reading config parameters");
  }
  std::cout << std::endl;
}

