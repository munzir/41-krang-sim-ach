# 41-krang-sim-ach
Hardware simulation of Krang on DART. Publishes states to and subscribes for inputs from ach channels on hardware. These channels existed primarily for communicating with the drivers on hardware. The simulation can be used instead of hardware drivers via these ach channels.

## Dependencies

- [DART](https://dartsim.github.io/install_dart_on_ubuntu.html) - Use `apt install` instructions on the page. Along with the basic library also install `dart6-utils-dev`, `dart6-gui-dev` and `dart6-utils-urdf-dev`
- [config4cpp](http://www.config4star.org/) - Using the link, download the source code for Config4Cpp (C++ version) using the 'compressed tar' option. After extracting, cd into the directory and

      make
  This creates files in bin, lib and include folders that we will manually copy to the system folder. Before doing so, make sure permissions of the files are set to `-rwxr-xr-x` for all files, and to `drwxr-xr-x` for the directory. You can check this using:

      ls -la
  inside the respective folders. This will list the files along with their permissions. If the permission is not the same as mentioned above, change the permission using chmod 755 <file-name>. For example, if doing "ls -la" in the include folder gives the output:

      drwx------ 2 munzir munzir 4096 Feb  2  2012 config4cpp
  then we need to change the permission for this folder using

      chmod 755 config4cpp
  Once the permissions are properly set, then copy files from `bin` folder to `/usr/local/bin/`, from `lib` folder to `/usr/local/lib/`, and the `config4cpp` directory in the include folder to `/usr/local/include`. Once the files are copied, do:

      sudo ldconfig
  for properly binding config4cpp with the dynamic-linker.
- [09-URDF](https://github.gatech.edu/WholeBodyControlAttempt1/09-URDF) - Clone. Inside the local clone, do:

      mkdir build
      cd build
      cmake ..
      sudo make install
- [18-OnlineCoM](https://github.gatech.edu/WholeBodyControlAttempt1/18-OnlineCoM) - Clone. Inside the local clone, do:

      mkdir build
      cd build
      cmake ..
      sudo make install
- [37-somatic](https://github.gatech.edu/WholeBodyControlAttempt1/37-somatic) - Follow installation instructions on the git readme.
- [42-joystick](https://github.gatech.edu/WholeBodyControlAttempt1/42-joystick) - Follow installation instructions on the git readme.
- [43-krang-waist](https://github.gatech.edu/WholeBodyControlAttempt1/43-krang-waist) - Follow installation instructions on the git readme.
- Update the dynamic linker, in case it is not automatically updated, by using:

      sudo ldconfig

## Installation

Follow the instructions:

    git clone https://github.gatech.edu/WholeBodyControlAttempt1/41-krang-sim-ach
    cd 41-krang-sim-ach
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make
    sudo make install

## Usage

In what follows, an alternate to `sudo service <program> start` is to do `sudo /etc/init.d/<program> start`. The former may sometimes not work only because the system has not been restarted after installation of `somatic` and `krang-sim-ach`.

### Start event logger
To use ach, we need to run the bash script in the `/etc/init.d` folder named `somatic`. It creates an events channels and logs the events (using a daemon `slogd`). These events are generated by other daemons/programs (such as our simulation program) communicating on ach. To run the script that starts the logger, do:

      sudo service somatic start

Note: Since this script lives in `/etc/init.d`, it is possible that the logger `slogd` is already running after system reboot. In that case, doing `somatic start` will give an error stating that it is already running. If this is the case, then you can move to the next step without doing anything.

### Create ach channels
To use the simulation along with its interface, you first need to launch the ach channels. This is done by the `krang-sim-interface` program:

    sudo service krang-sim-interface create

### Launch the simulation
Now you are ready to launch the simulation

    sudo krang-sim-ach

When the window is launched, press [Space] to start simulation. Press 'v' to get rid of the annoying lines on the simulation at body contact points. You can press [Esc] to kill the simulation window. If you want only the simulation to run without rendering set the parameter `render' in `/usr/local/share/krang-sim-ach/cfg/dart_params.cfg` to `"false"`. To enable/disable external timestepping, modify the parameter `external_timestepping` in `/usr/local/share/krang-sim-ach/ach_params.cfg`. These changes will only take effect when the simulation is launched again.

### Delete ach channels
To close the ach channels:

    sudo service krang-sim-interface delete
    
You don't need to delete ach channels every time you re-launch the simulation. They need to be created only once. And then simulation can be killed and re-launched withou the need to delete/re-create ach channels.

### Stop event logger
To delete the events channel and stop the logging:

      sudo service somatic stop

## Example program

An example program that can interface with this simulation is found in repo [35-balancing](https://github.gatech.edu/WholeBodyControlAttempt1/35-balancing). This program needs two other programs to run (joystick and krang-waist) in order to do its job. If you have a the physical joystick transceiver plugged in to your system, then you can launch these two programs using the following command. This is assuming that the event logger is already alive:

    sudo service krang-sim-interface start

This will not only launch those programs but also create ach channels if they are not already created.

To kill the programs:

    sudo service krang-sim-interface stop

This will also delete the ach channels that were created. It will not delete the event channel being used for logging (see previous section).
