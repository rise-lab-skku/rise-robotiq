#!/usr/bin/env python3

"""
This script is a modified version of the Robotiq3FGripperRtuNode.py script from the robotiq_3f_gripper_control package.
"""

import baseRobotiq3FGripper
import robotiq_modbus_rtu.comModbusRtu
import sys

from Robotiq3FGripperControl import Robotiq3FGripperControl
from GripperIO import Robotiq3FMode


def mainLoop(address: str = "/dev/ttyUSB0"):
    # Gripper is a 3F gripper with a TCP connection
    gripper = baseRobotiq3FGripper.robotiqbaseRobotiq3FGripper()
    gripper.client = robotiq_modbus_rtu.comModbusRtu.communication(retry=True)

    # We connect to the address received as an argument
    try:
        is_connected = gripper.client.connectToDevice(address)
        assert is_connected, f"Cannot connect to device: {address}"
    except Exception as e:
        print(e)
        print("Please check the wire connection or permission of the port.")
        print("Example on how to run this script:")
        print("  python3 NoRosRtuNode.py /dev/ttyUSB0")

    ##########################################

    control = Robotiq3FGripperControl(gripper)
    control.reset()
    assert control.wait_for_connection(), "Cannot connect to gripper"
    print("Gripper connected")
    print(f"\nGripper status:\n{control.status}")

    control.initialize(speed=255, force=255)
    print(f"\nGripper command:\n{control.cmd}\n")
    control.print_mode()
    control.wait_until_stop()
    print("Initialization completed.\n")

    ##########################################

    print("Now, we are ready to send commands to the gripper.\n")
    control.set_target_force(100)

    for mode in Robotiq3FMode:
        print(f"Press [ENTER] to change the mode to {mode.name}.")
        input()
        # For example, to change the mode to WIDE_MODE:
        #   control.set_mode(Robotiq3FMode.WIDE_MODE)
        control.set_mode(mode)
        control.go()
        control.print_mode()
        control.wait_until_stop()

        print("Press [ENTER] to CLOSE the gripper.")
        input()
        control.simple_close()

        print("Press [ENTER] to OPEN the gripper.")
        input()
        control.simple_open()

    ##########################################

    print("Back to BASIC_MODE and close the gripper to finish.")
    control.set_mode(Robotiq3FMode.BASIC_MODE)
    control.simple_close()


if __name__ == "__main__":
    """
    Usage:
        python3 NoRosRtuNode.py /dev/ttyUSB0
    """
    mainLoop(sys.argv[1])
