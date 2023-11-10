# Robotiq
![robotiq_3f](https://img.shields.io/badge/ROBOTIQ_3f-red)

![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-green)
![ROS](https://img.shields.io/badge/ROS-noetic-yellow)
![Python](https://img.shields.io/badge/Python-3.x-blue)

## RISE patch note

* Long finger type robotiq 2f grippper is added from `melodic-devel` branch
* Robotiq E-Pick gripper is added from `melodic-devel` branch


* original robotiq_3f gripper is not officially supported. especially, Robotiq3FGripperRtuNode.py is gripper driver ros node. This is necessary when tgrying to operate the gripper using ros.
* when you want to trying to operate the gripper using ros.


```sh
rosrun robotiq_3f_gripper_control Robotiq3FGripperRtuNode.py /dev/ttyUSB0

rosrun robotiq_3f_gripper_control Robotiq3FGripperSimpleController.py
```

### Trouble Shooting

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
