# Project Capture | Claw Control
## Description
This repository is part of **Project Capture** ([website](https://capture.isr.tecnico.ulisboa.pt/pt/)) focused on the development of claw control and design of the claw, drone and map for simulations in Gazebo Classic.

## Running
To start de simulation in gazebo:
```bash
ros2 launch capture capture.launch.py
```
To start the pegasus console:
```bash
ros2 run pegasus_console pegasus_console
```

## Installation
### Installing Pegasus & Capture Project | Claw Control
1. Before installing the Capture Project you need to add the pegasus external workspace, according to the [Pegasus Installation Guide](https://pegasusresearch.github.io/pegasus/source/setup/installation.html).
```bash
mkdir -p ~/pegasus_external
cd ~/pegasus_external

# Install the dependencies for CasADi (IPOPT)
sudo apt install coinor-libipopt-dev python3-pip

# Clone the repository (SSH)
git clone git@github.com:PegasusResearch/pegasus_external.git src

# Compile the code
colcon build --symlink-install

echo "source ~/pegasus_external/install/setup.bash" >> ~/.bashrc
```
2. The next step is to install the Project Capture | Claw Control.

```bash
# Create the workspace
mkdir -p ~/capture/src
cd ~/capture/src

# Clone the repository (SSH)
git clone git@github.com:abrantessss/capture.git --recursive

# Go to the workspace
cd ~/capture

# Compile the code
colcon build --symlink-install

# Add the source to your .bashrc
echo "source ~/capture/install/setup.bash" >> ~/.bashrc
```

### Installing Gazebo Classic & PX4-Autopilot
1. Start by installing Gazebo Classic.
```bash
# Make sure to uninstall gazebo garden (if installed)
sudo apt remove gz-garden
sudo apt install aptitude

# Install gazebo (classic)
sudo aptitude install gazebo libgazebo11 libgazebo-dev

# Install the ROS packages for gazebo
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs
```

2. Install the PX4-Autopilot used for handling motor control and sensor integration.

Install the dependencies (to be able to compile PX4-Autopilot):
```bash
# Linux packages
sudo apt install git make cmake python3-pip

# Python packages
pip install kconfiglib jinja2 empy jsonschema pyros-genmsg packaging toml numpy future

# GStreamer (for video streaming)
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
```

Clone the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot):
```bash
# Option 1: With HTTPS
git clone https://github.com/PX4/PX4-Autopilot.git
# Option 2: With SSH (you need to setup a github account with ssh keys)
git clone git@github.com:PX4/PX4-Autopilot.git
```

Checkout to the stable version 1.14.3 and compile the code for software-in-the-loop (SITL) mode:
```bash
# Go to the PX4 directory
cd PX4-Autopilot

# Checkout to the latest stable release
git checkout v1.14.3

# Compile the code in SITL mode
make px4_sitl gazebo-classic
```

Add the following line to your .bashrc file:
```bash
echo "export PX4_DIR=$(pwd)" >> ~/.bashrc
```

## Further Steps 
### PX4-Autopilot
In PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes create a new airframe "_10020_gazebo-classic_pegasus_capture_".

```
	#!/bin/sh
#
# @name Capture (MB690B Model)
#
# @type Quadrotor Wide
#
# @maintainer Luís Abrantes <luis.g.abrantes@tecnico.ulisboa.pt>
#
# 3DR Iris Quadrotor SITL of Julian Oes <julian@oes.ch>

. ${R}etc/init.d/rc.mc_defaults


param set-default CA_AIRFRAME 0

param set-default CA_ROTOR_COUNT 4
param set-default CA_ROTOR0_PX 0.1515
param set-default CA_ROTOR0_PY 0.245
param set-default CA_ROTOR0_KM 0.05
param set-default CA_ROTOR1_PX -0.1515
param set-default CA_ROTOR1_PY -0.1875
param set-default CA_ROTOR1_KM 0.05
param set-default CA_ROTOR2_PX 0.1515
param set-default CA_ROTOR2_PY -0.245
param set-default CA_ROTOR2_KM -0.05
param set-default CA_ROTOR3_PX -0.1515
param set-default CA_ROTOR3_PY 0.1875
param set-default CA_ROTOR3_KM -0.05

param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104
```
