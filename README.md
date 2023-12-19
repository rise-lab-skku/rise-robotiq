# Robotiq

![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-green)
![ROS](https://img.shields.io/badge/ROS-noetic-yellow)
![Python](https://img.shields.io/badge/Python-3.x-blue)

## RISE's Patch Notes

* Long finger type Robotiq-2F is added from `melodic-devel` branch ([e456f6f](https://github.com/rise-lab-skku/rise-robotiq/commit/e456f6f3de6e85cf021da7db733555a3806245fc))
* Robotiq E-Pick is added from `melodic-devel` branch ([b850dd2](https://github.com/rise-lab-skku/rise-robotiq/commit/b850dd2769ce4706edd9c8a670bd1891f3b67457))
* Python interface of Robotiq-3F is added from CLI interface ([c11b925](https://github.com/rise-lab-skku/rise-robotiq/commit/c11b92597cf7cd14be71b4e9bd5e28bbb253d43d))
* No-ROS version of Robotiq-3F is added ()

## How to install

We recommend building only the packages you need.
We are using `catkin build`, but `catkin_make` is also fine.

```sh
catkin build $TARGET_PACKAGE
```

Some packages require additional dependencies.

## How to use

### Robotiq 3F

```sh
# Robotiq3FGripperRtuNode.py makes connection with gripper.
rosrun robotiq_3f_gripper_control Robotiq3FGripperRtuNode.py /dev/ttyUSB0

# Robotiq3FGripperSimpleController.py can control gripper using keyboard.
rosrun robotiq_3f_gripper_control Robotiq3FGripperSimpleController.py
```

## Troubleshooting

* Check your pymodbus version

   ```sh
   pip install -U pymodbus==2.5.3
   ```

* Add permissions

   ```bash
   sudo usermod -a -G dialout $USER
   ```

   ```bash
   sudo chmod 777 /dev/ttyUSB0
   ```

## ROS1 Distro Support

| Distro | Branch | Reference |
| :----: |:-----: |:--------: |
| Kinetic | [indigo-devel](https://github.com/rise-lab-skku/rise-robotiq/tree/kinetic-devel) | [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq) |
| Melodic | [melodic-devel](https://github.com/rise-lab-skku/rise-robotiq/tree/melodic-devel) | [rise-lab-skku/rise-robotiq](https://github.com/rise-lab-skku/rise-robotiq) |
| Noetic | [noetic-devel](https://github.com/rise-lab-skku/rise-robotiq/tree/noetic-devel) | [TAMS-Group/robotiq](https://github.com/TAMS-Group/robotiq) |

## License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Contents

This repo holds source code for all versions > groovy. For those versions <= groovy see: [SVN repo][]

[ROS wiki]: http://ros.org/wiki/robotiq
[SVN repo]: https://code.google.com/p/swri-ros-pkg/source/browse
