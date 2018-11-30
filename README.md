# 41-krang-sim-ach
Hardware simulation of Krang on DART. Publishes states to and subscribes for inputs from ach channels on hardware. These channels existed primarily for communicating with the drivers on hardware. The simulation can be used instead of hardware drivers via these ach channels.

## Dependencies

- [DART](https://dartsim.github.io/install_dart_on_ubuntu.html) - Use 'apt install' instructions on the page.
- [09-URDF](https://github.gatech.edu/WholeBodyControlAttempt1/09-URDF) - Clone this repo. No installation needed.
- [18-OnlineCoM](https://github.gatech.edu/WholeBodyControlAttempt1/18-OnlineCoM) - Clone this repo. No installation needed.
- [37-somatic](https://github.gatech.edu/WholeBodyControlAttempt1/37-somatic) - Follow installation instructions on the git readme.
- [42-joystick](https://github.gatech.edu/WholeBodyControlAttempt1/42-joystick) - Follow installation instructions on the git readme.
- [43-krang-waist](https://github.gatech.edu/WholeBodyControlAttempt1/43-krang-waist) - Follow installation instructions on the git readme.

## Installation

Clone the repo.

    git clone https://github.gatech.edu/WholeBodyControlAttempt1/41-krang-sim-ach

Assuming you are in the newly created directory 41-krang-sim-ach, you will notice a folder named cfg. In it, there is a configuration file with the name dart_params.cfg. Open this file and notice that the first two parameters are krang_urdf_path and com_params_path. The parameter krang_urdf_path is the path to the URDF of the robot. This URDF is found in repo 09-URDF. Specify the absolute path to the local location of this file on your system. Similarly, the second parameter com_params_path is the path to dynamic parameters of the robot saved in repo 18-OnlineCoM. Again, specify the absolute path to the local location of this file on your system. 

Next create a folder named krang-sim-ach in system folder /usr/local/share. And copy the cfg folder to /usr/local/share/krang-sim-ach. Assuming you are in directory 41-krang-sim-ach, follow the steps:

    sudo mkdir /usr/local/share/krang-sim-ach
    sudo cp -r cfg /usr/local/share/krang-sim-ach

Finally, compile the code. Assuming you are currently in 41-krang-sim-ach, follow the steps:

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make

## Usage

To use the simulation along with its interface, you first need to launch the ach channels. There is a bash script in the bash folder that takes care of this. Assuming you are in 41-krang-sim-ach, follow the steps:

    cd bash
    sudo service somatic start
    sudo ./krang-sim-interface start

Now you are ready to launch the simulation. Assuming you are in 41-krang-sim-ach, follow the steps:

    cd build
    sudo ./krang-sim-ach

When the window is launched, press spacebar to start simulation. Press 'v' to get rid of the annoying lines on the simulation at body contact points. An example program that can interface with this simulation is found in repo [35-balancing](https://github.gatech.edu/WholeBodyControlAttempt1/35-balancing). You can press 'Esc' to kill the simulation window.

To close the ach channels and somatic, follow the steps (assuming you are in 41-krang-sim-ach):

    cd bash
    sudo ./krang-sim-interface stop
    sudo service somatic stop
