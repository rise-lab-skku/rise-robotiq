#!/usr/bin/env python3

# Warning. Naming is wired! (Input: Status, Output: Command)
from GripperIO import Robotiq3FGripperRobotInput as RtqStatus
from GripperIO import Robotiq3FGripperRobotOutput as RtqCommand

from GripperIO import Robotiq3FMode as MODE
from GripperIO import Robotiq3FFinger as FINGER

from baseRobotiq3FGripper import robotiqbaseRobotiq3FGripper
import time


def prevent_stale_data(func):
    def wrapper(*args, **kwargs):
        args[0].update_status()
        return func(*args, **kwargs)

    return wrapper


class Robotiq3FGripperControl(object):
    def __init__(self, gripper: robotiqbaseRobotiq3FGripper):
        self._gripper = gripper
        self._last_status_update_time = time.time()
        self._status_update_cooldown = 0.1
        self.status = None
        self.cmd = RtqCommand()

    #########################################################
    # Command
    #########################################################

    def deactivate(self):
        self.cmd.rACT = 0

    def activate(self):
        self.cmd.rACT = 1

    def set_mode(self, mode: MODE):
        self.cmd.rMOD = int(mode)

    def set_go_to_action(self, on: bool):
        self.cmd.rGTO = 1 if on else 0

    def set_automatic_emergency_release(self, on: bool):
        self.cmd.rATR = 1 if on else 0

    def set_glove_mode(self, on: bool):
        self.cmd.rGLV = 1 if on else 0

    def set_target_position(self, pos: int, finger=FINGER.ALL_FINGERS):
        # pos: open = 0, close = 255
        if finger == FINGER.ALL_FINGERS:
            self.cmd.rICF = 0
            self.cmd.rPRA = pos
        elif finger == FINGER.FINGER_A:
            self.cmd.rICF = 1
            self.cmd.rPRA = pos
        elif finger == FINGER.FINGER_B:
            self.cmd.rICF = 1
            self.cmd.rPRB = pos
        elif finger == FINGER.FINGER_C:
            self.cmd.rICF = 1
            self.cmd.rPRC = pos
        elif finger == FINGER.SCISSOR:
            self.cmd.rICS = 1
            self.cmd.rPRS = pos

    def set_target_speed(self, speed: int, finger=FINGER.ALL_FINGERS):
        # speed: 22 mm/s = 0, 110 mm/s = 255
        if finger == FINGER.ALL_FINGERS:
            self.cmd.rICF = 0
            self.cmd.rSPA = speed
        elif finger == FINGER.FINGER_A:
            self.cmd.rICF = 1
            self.cmd.rSPA = speed
        elif finger == FINGER.FINGER_B:
            self.cmd.rICF = 1
            self.cmd.rSPB = speed
        elif finger == FINGER.FINGER_C:
            self.cmd.rICF = 1
            self.cmd.rSPC = speed
        elif finger == FINGER.SCISSOR:
            self.cmd.rICS = 1
            self.cmd.rSPS = speed

    def set_target_force(self, force: int, finger=FINGER.ALL_FINGERS):
        # force: 15 N = 0, 60 N = 255
        if finger == FINGER.ALL_FINGERS:
            self.cmd.rICF = 0
            self.cmd.rFRA = force
        elif finger == FINGER.FINGER_A:
            self.cmd.rICF = 1
            self.cmd.rFRA = force
        elif finger == FINGER.FINGER_B:
            self.cmd.rICF = 1
            self.cmd.rFRB = force
        elif finger == FINGER.FINGER_C:
            self.cmd.rICF = 1
            self.cmd.rFRC = force
        elif finger == FINGER.SCISSOR:
            self.cmd.rICS = 1
            self.cmd.rFRS = force

    #########################################################
    # Status
    #########################################################

    def update_status(self) -> RtqStatus:
        now = time.time()
        if now - self._last_status_update_time > self._status_update_cooldown:
            self.status = self._gripper.getStatus()
            self._last_status_update_time = now
        return self.status

    @prevent_stale_data
    def is_reset(self):
        return self.status.gGTO == 0

    @prevent_stale_data
    def is_in_motion(self):
        return self.status.gGTO == 1

    @prevent_stale_data
    def is_ready(self):
        return self.status.gACT == 1

    @prevent_stale_data
    def get_mode(self) -> int:
        return self.status.gMOD

    def print_mode(self):
        mode = MODE(self.get_mode())
        print(f"Current mode: {mode.name}")
        for m in MODE:
            chk = "v" if m == mode else " "
            print(f"  [{chk}] {m.name}: {m.value}")

    @prevent_stale_data
    def get_gripper_status(self):
        gripper_status = self.status.gIMC
        if gripper_status == 0:
            print("Gripper is in reset (or automatic release) state. See Fault status if Gripper is activated.")
        elif gripper_status == 1:
            print("Activation is in progress.")
        elif gripper_status == 2:
            print("Mode change is in progress.")
        else:
            print("Activation and mode change are completed.")
        return self.status.gIMC

    @prevent_stale_data
    def get_motion_status(self):
        motion_status = self.status.gSTA
        if motion_status == 0:
            if self.is_in_motion():
                print("Gripper is in motion towards requested position (now girpper is operating)")
            else:
                print("Action status is False. if you want to watching Gripper motion. Please set Activation")
        elif motion_status == 1:
            print("Gripper is stopped. One or two fingers stopped before requested position")
        elif motion_status == 2:
            print("Gripper is stopped. All fingers stopped before requested position")
        else:
            print("Gripper is stopped. All fingers reached requested position")
        return motion_status

    def _explain_finger_status(self, finger_name: str, finger_status: int):
        if finger_status == 0:
            if self.is_in_motion():
                print(f"{finger_name} is in motion (now girpper is operating).")
            else:
                print(f"Action status is False. if you want to watching {finger_name} motion. Please set Activation")
        elif finger_status == 1:
            print(f"{finger_name} has stopped due to a contact while opening.")
        elif finger_status == 2:
            print(f"{finger_name} has stopped due to a contact while closing.")
        else:
            print(f"{finger_name} is at requested position.")

    @prevent_stale_data
    def get_finger_A_status(self):
        self._explain_finger_status("FINGER_A", self.status.gDTA)
        return self.status.gDTA

    @prevent_stale_data
    def get_finger_B_status(self):
        self._explain_finger_status("FINGER_B", self.status.gDTB)
        return self.status.gDTB

    @prevent_stale_data
    def get_finger_C_status(self):
        self._explain_finger_status("FINGER_C", self.status.gDTC)
        return self.status.gDTC

    @prevent_stale_data
    def get_scissor_status(self):
        self._explain_finger_status("SCISSOR", self.status.gDTS)
        return self.status.gDTS

    @prevent_stale_data
    def get_fault_status(self):
        fault_status = self.status.gFLT
        if fault_status == 0:
            print("No fault (fault LED off)")
        elif fault_status == 5:
            print("Priority faults (fault LED off)\n")
            print("Action delayed, activation (reactivation) must be completed prior to action.")
        elif fault_status == 6:
            print("Priority faults (fault LED off)\n")
            print("Action delayed, mode change must be completed prior to action.")
        elif fault_status == 7:
            print("Priority faults (fault LED off)\n")
            print("The activation bit must be set prior to action.")
        elif fault_status == 9:
            print("Minor faults (fault LED continuous red)\n")
            print("The communication chip is not ready (may be booting).")
        elif fault_status == 10:
            print("Minor faults (fault LED continuous red)\n")
            print("Changing mode fault, interferences detected on Scissor (for less than 20 sec).")
        elif fault_status == 11:
            print("Minor faults (fault LED continuous red)\n")
            print("Automatic release in progress.")
        elif fault_status == 13:
            print("Major faults (fault LED blinking red) - Reset is required\n")
            print("Activation fault, verify that no interference or other error occurred.")
        elif fault_status == 14:
            print("Major faults (fault LED blinking red) - Reset is required\n")
            print("Changing mode fault, interferences detected on Scissor (for more than 20 sec).")
        elif fault_status == 15:
            print("Major faults (fault LED blinking red) - Reset is required\n")
            print("Automatic release completed. Reset and activation is required.")
        return fault_status

    def _explain_position(self, name: str, pos: int) -> int:
        if pos == 0:
            print(f"{name} is at minimum position (full opening)")
        elif pos == 255:
            print(f"{name} is at maximum position (full closing)")
        else:
            print(f"{name} is {pos}")
        return pos

    def _explain_target_finger_pos(self, finger_name: str, target_pos: int) -> int:
        if self.cmd.rICF:
            return self._explain_position(finger_name, target_pos)
        else:
            print(f"Individual mode is OFF. All fingers have same target position: {self.status.gPRA}")
            return self.status.gPRA

    @prevent_stale_data
    def get_gripper_target_pos(self) -> int:
        if self.cmd.rICF:
            print("Individual mode is ON")
            return -1
        else:
            return self._explain_position("GRIPPER target position", self.status.gPRA)

    @prevent_stale_data
    def get_finger_A_target_pos(self) -> int:
        return self._explain_target_finger_pos("FINGER_A target position", self.status.gPRA)

    @prevent_stale_data
    def get_finger_B_target_pos(self):
        return self._explain_target_finger_pos("FINGER_B target position", self.status.gPRB)

    @prevent_stale_data
    def get_finger_C_target_pos(self):
        return self._explain_target_finger_pos("FINGER_C target position", self.status.gPRC)

    @prevent_stale_data
    def get_scissor_target_pos(self):
        return self._explain_target_finger_pos("SCISSOR target position", self.status.gPRS)

    @prevent_stale_data
    def get_gripper_actual_pos(self):
        if self.cmd.rICF:
            print("Individual mode is ON")
            return -1
        else:
            return self._explain_position("GRIPPER actual position", self.status.gPOA)

    @prevent_stale_data
    def get_finger_A_actual_pos(self):
        return self._explain_position("FINGER_A actual position", self.status.gPOA)

    @prevent_stale_data
    def get_finger_B_actual_pos(self):
        return self._explain_position("FINGER_B actual position", self.status.gPOB)

    @prevent_stale_data
    def get_finger_C_actual_pos(self):
        return self._explain_position("FINGER_C actual position", self.status.gPOC)

    @prevent_stale_data
    def get_scissor_actual_pos(self):
        return self._explain_position("SCISSOR actual position", self.status.gPOS)

    def _explain_current_consumption(self, name: str, current: int) -> int:
        approx_current = current * 0.1
        print(f"{name} current consumption is {current}. Approximately {approx_current} mA")
        return current

    @prevent_stale_data
    def get_finger_A_current_consumption(self):
        return self._explain_current_consumption("FINGER_A", self.status.gCUA)

    @prevent_stale_data
    def get_finger_B_current_consumption(self):
        return self._explain_current_consumption("FINGER_B", self.status.gCUB)

    @prevent_stale_data
    def get_finger_C_current_consumption(self):
        return self._explain_current_consumption("FINGER_C", self.status.gCUC)

    @prevent_stale_data
    def get_scissor_current_consumption(self):
        return self._explain_current_consumption("SCISSOR", self.status.gCUS)

    #########################################################
    # Utilities
    #########################################################

    @prevent_stale_data
    def is_stopped(self) -> bool:
        return self.status.gSTA and self.status.gDTA and self.status.gDTB and self.status.gDTC

    def is_moving(self) -> bool:
        return not self.is_stopped()

    @prevent_stale_data
    def is_target_reached(self) -> bool:
        target = self.cmd.rPRA if self.cmd.rICF else (self.cmd.rPRA, self.cmd.rPRB, self.cmd.rPRC)
        now = self.status.gPOA if self.cmd.rICF else (self.status.gPOA, self.status.gPOB, self.status.gPOC)
        return target == now

    def _wait_for_condition(self, condition, timeout=10, check_interval=0.1, message: str = None) -> bool:
        start_time = time.time()
        while True:
            time.sleep(check_interval)
            self.update_status()
            if condition():
                return True
            if time.time() - start_time > timeout:
                if message is not None:
                    print(f"Timeout({timeout}s) reached while waiting for {message}")
                return False

    def wait_for_connection(self, timeout=10) -> bool:
        def is_connected():
            return self.update_status() is not None

        return self._wait_for_condition(is_connected, timeout=timeout, message="connection")

    def wait_until_stop(self, timeout=10) -> bool:
        return self._wait_for_condition(self.is_stopped, timeout=timeout, message="stop")

    def wait_until_moving(self, timeout=-1):
        return self._wait_for_condition(self.is_moving, timeout=timeout, message="moving")

    def reset(self):
        self.cmd = RtqCommand()
        self.deactivate()
        self.send_command(self.cmd)

    def initialize(self, speed=200, force=200):
        self.activate()
        self.set_target_speed(speed)
        self.set_target_force(force)
        self.go()
        print("Gripper is initializing...")
        time.sleep(15)

    def simple_close(self):
        self.set_target_position(255)
        self.go(wait_until_stop=True)

    def simple_open(self):
        self.set_target_position(0)
        self.go(wait_until_stop=True)

    def send_command(self, command: RtqCommand):
        self._gripper.refreshCommand(command)
        self._gripper.sendCommand()

    def go(self, sleep=0.1, wait_until_stop=False):
        self.set_go_to_action(True)
        self.send_command(self.cmd)
        if wait_until_stop:
            self.wait_until_stop(10000)
        else:
            time.sleep(sleep)
