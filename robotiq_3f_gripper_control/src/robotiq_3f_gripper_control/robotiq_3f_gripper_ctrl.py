#!/usr/bin/env python3

import numpy as np
import rospy
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotInput as inputMsg
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput as outputMsg


class Robotiq3FGripperCTRL(object):
    def __init__(self):
        self.cur_status = None
        self.status_sub = rospy.Subscriber("Robotiq3FGripperRobotInput",
                                           inputMsg,
                                           self._status_cb
                                            )
        self.cmd_pub = rospy.Publisher("Robotiq3FGripperRobotOutput",
                                       outputMsg,
                                       queue_size=2
                                       )

        self.message = []

        self.outputMsg = outputMsg()

    def _status_cb(self, msg):
        self.cur_status = msg

    #######################################         output msg            ######################################

    # rACT: Action request (activation bit).
    # 0x0 - Decativate gripper.
    # 0x1 - Activate gripper.

    def deactive(self):
        self.outputMsg.rACT = 0

    def active(self):
        self.outputMsg.rACT = 1

    # rMOD: Changes the gripper grasping mode.
    # 0x0 - Basic mode.
    # 0x1 - Pinch mode.
    # 0x2 - Wide mode.
    # 0x3 - Scissor mode.

    def basic_mode(self):
        self.outputMsg.rMOD = 0

    def pinch_mode(self):
        self.outputMsg.rMOD = 1

    def wide_mode(self):
        self.outputMsg.rMOD = 2

    def scissor_mode(self):
        self.outputMsg.rMOD = 3

    # rGTO: "Go To" action moves the gripper fingers to the requested position.
    # 0x0 - Stop.
    # 0x1 - Go to the requested position.

    def stop(self):
        self.outputMsg.rGTO = 0

    def go_to_request(self):
        self.outputMsg.rGTO = 1

    # rGLV: Glove mode
    # 0x0 - Glove mode off
    # 0x1 - Glove mode on

    def glove_mode_off(self):
        self.outputMsg.rGLV = 0

    def glove_mode_on(self):
        self.outputMsg.rGLV = 1

    # rPRA: Target position of the fingers (or finger A only if bit rICF is set).
    # 0x00 Minimum position (open).
    # 0xFF Maximum position (close).

    def set_target_pos(self,pos):# pos : 0~255
        self.outputMsg.rICF = 0
        self.outputMsg.rPRA = pos

    # rSPA: Gripper closing or opening speed (or finger A only if bit rICF is set).
    # Setting a speed will not initiate a motion.
    # 0x00 Minimum speed.
    # 0xFF Maximum speed.

    def set_grasping_speed(self,speed):# speed : 0~255
        self.outputMsg.rICF = 0
        self.outputMsg.rSPA = speed

    # rFRA: Final grasping force of the gripper (or finger A only if bit rICF is
    # set).
    # 0x00 Minimum force.
    # 0xFF Maximum force.

    def set_grasping_force(self,force):# force : 0~255
        self.outputMsg.rICF = 0
        self.outputMsg.rFRA = force

    # rPRA: Target position of the fingers (or finger A only if bit rICF is set).
    # 0x00 Minimum position (open).
    # 0xFF Maximum position (close).

    def set_finger_a_target_pos(self,pos):# pos : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rPRA = pos

    # rSPA: Gripper closing or opening speed (or finger A only if bit rICF is set).
    # Setting a speed will not initiate a motion.
    # 0x00 Minimum speed.
    # 0xFF Maximum speed.

    def set_finger_a_target_grasping_speed(self,speed):# speed : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rSPA = speed

    # rFRA: Final grasping force of the gripper (or finger A only if bit rICF is
    # set).
    # 0x00 Minimum force.
    # 0xFF Maximum force.

    def set_finger_a_grasping_force(self,force):# force : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rFRA = force

    # rPRB: Finger B target position. It is only available if bit rICF is set.
    # 0x00 Minimum position (open).
    # 0xFF Maximum position (close).

    def set_finger_b_target_pos(self,pos):# pos : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rPRB = pos

    # rSPB: Finger B speed. It is only available if bit rICF is set.
    # Setting a speed will not initiate a motion.
    # 0x00 Minimum speed.
    # 0xFF Maximum speed.

    def set_finger_b_target_speed(self,speed):# speed : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rSPB = speed

    # rFRB: Finger B force. It is only available if bit rICF is set.
    # 0x00 Minimum force.
    # 0xFF Maximum force.

    def set_finger_b_grasping_force(self,force):# force : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rFRB = force

    # rPRC: Finger C target position. It is only available if bit rICF is set.
    # 0x00 Minimum position (open).
    # 0xFF Maximum position (close).

    def set_finger_c_target_pos(self,pos):# pos : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rPRC = pos

    # rSPC: Finger C speed. It is only available if bit rICF is set.
    # Setting a speed will not initiate a motion.
    # 0x00 Minimum speed.
    # 0xFF Maximum speed.

    def set_finger_c_target_speed(self,speed):# speed : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rSPC = speed

    # rFRC: Finger C force. It is only available if bit rICF is set.
    # 0x00 Minimum force.
    # 0xFF Maximum force.

    def set_finger_c_grasping_force(self,force):# force : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rFRC = force

    # rPRS: Scissor axis target position. It is only applied if the Individual
    # Control of Scissor option is selected (bit rICS is set).
    # 0x00 Minimum position (open).
    # 0xFF Maximum position (close).

    def set_scissor_axis_target_pos(self,pos):# pos : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rPRS = pos

    # rSPS: Scissor axis speed. It is only applied if the Individual
    # Control of Scissor option is selected (bit rICS is set).
    # 0x00 Minimum speed.
    # 0xFF Maximum speed.

    def set_scissor_axis_target_speed(self,speed):# speed : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rSPS = speed

    # rFRS: Scissor axis force. It is only applied if the Individual
    # Control of Scissor option is selected (bit rICS is set).
    # 0x00 Minimum force.
    # 0xFF Maximum force.

    def set_scissor_axis_target_force(self,force):# force : 0~255
        self.outputMsg.rICF = 1
        self.outputMsg.rFRS = force


    ############################################################################################################

    #######################################          input msg            ######################################

    def is_reset(self):
        return self.cur_status.gGTO == 0

    def is_action(self):
        return self.cur_status.gGTO == 1

    def is_ready(self):
        return self.cur_status.gACT == 1

    def get_mode(self):
        mode = self.cur_status.gMOD
        if mode == 0:
            print('Basic mode')
        elif mode == 1:
            print('Pinch mode')
        elif mode == 2:
            print('Wide mode')
        else:
            print('Scissor mode')
        return self.cur_status.gMOD


    def get_gripper_status(self):
        gripper_status = self.cur_status.gIMC
        if gripper_status == 0:
            print('Gripper is in reset (or automatic release) state. See Fault status if Gripper is activated.')
        elif gripper_status == 1:
            print('Activation is in progress.')
        elif gripper_status == 2:
            print('Mode change is in progress.')
        else:
            print('Activation and mode change are completed.')
        return self.cur_status.gIMC

    def get_motion_status(self):
        motion_status = self.cur_status.gSTA
        if motion_status == 0:
            if self.is_action():
                print('Gripper is in motion towards requested position (now girpper is operating)')
            else:
                print('Action status is False. if you want to watching Gripper motion. Please set Activation')
        elif motion_status == 1:
            print('Gripper is stopped. One or two fingers stopped before requested position')
        elif motion_status == 2:
            print('Gripper is stopped. All fingers stopped before requested position')
        else:
            print('Gripper is stopped. All fingers reached requested position')
        return self.cur_status.gSTA

    def get_finger_a_status(self):
        finger_a_status = self.cur_status.gDTA
        if finger_a_status == 0:
            if self.is_action():
                print('Finger A is in motion (now girpper is operating).')
            else:
                print('Action status is False. if you want to watching Finger A motion. Please set Activation')
        elif finger_a_status == 1:
            print('Finger A has stopped due to a contact while opening.')
        elif finger_a_status == 2:
            print('Finger A has stopped due to a contact while closing.')
        else:
            print('Finger A is at requested position.')
        return self.cur_status.gDTA

    def get_finger_b_status(self):
        finger_b_status = self.cur_status.gDTB
        if finger_b_status == 0:
            if self.is_action():
                print('Finger B is in motion (now girpper is operating).')
            else:
                print('Action status is False. if you want to watching Finger B motion. Please set Activation')
        elif finger_b_status == 1:
            print('Finger B has stopped due to a contact while opening.')
        elif finger_b_status == 2:
            print('Finger B has stopped due to a contact while closing.')
        else:
            print('Finger B is at requested position.')
        return self.cur_status.gDTB

    def get_finger_c_status(self):
        finger_c_status = self.cur_status.gDTC
        if finger_c_status == 0:
            if self.is_action():
                print('Finger C is in motion (now girpper is operating).')
            else:
                print('Action status is False. if you want to watching Finger C motion. Please set Activation')
        elif finger_c_status == 1:
            print('Finger C has stopped due to a contact while opening.')
        elif finger_c_status == 2:
            print('Finger C has stopped due to a contact while closing.')
        else:
            print('Finger C is at requested position.')
        return self.cur_status.gDTC

    def get_scissor_status(self):
        scissor_status = self.cur_status.gDTS
        if scissor_status == 0:
            if self.is_action():
                print('Scissor is in motion (now girpper is operating).')
            else:
                print('Action status is False. if you want to watching Scissor motion. Please set Activation')
        elif scissor_status == 1:
            print('Scissor has stopped due to a contact while opening.')
        elif scissor_status == 2:
            print('Scissor has stopped due to a contact while closing.')
        else:
            print('Scissor is at requested position.')
        return self.cur_status.gDTS

    def get_err_msg(self):
        err = self.cur_status.gFLT
        if err == 0:
            print('No fault (fault LED off)')
        elif err == 5:
            print('Priority faults (fault LED off)\n')
            print('Action delayed, activation (reactivation) must be completed prior to action.')
        elif err == 6:
            print('Priority faults (fault LED off)\n')
            print('Action delayed, mode change must be completed prior to action.')
        elif err == 7:
            print('Priority faults (fault LED off)\n')
            print('The activation bit must be set prior to action.')
        elif err == 9:
            print('Minor faults (fault LED continuous red)\n')
            print('The communication chip is not ready (may be booting).')
        elif err ==10:
            print('Minor faults (fault LED continuous red)\n')
            print('Changing mode fault, interferences detected on Scissor (for less than 20 sec).')
        elif err == 11:
            print('Minor faults (fault LED continuous red)\n')
            print('Automatic release in progress.')
        elif err == 13:
            print('Major faults (fault LED blinking red) - Reset is required\n')
            print('Activation fault, verify that no interference or other error occurred.')
        elif err == 14:
            print('Major faults (fault LED blinking red) - Reset is required\n')
            print('Changing mode fault, interferences detected on Scissor (for more than 20 sec).')
        elif err == 15:
            print('Major faults (fault LED blinking red) - Reset is required\n')
            print('Automatic release completed. Reset and activation is required.')
        return self.cur_status.gFLT


    def get_gripper_target_pos(self):
        if self.outputMsg.rICF == 0:
            target_pos = self.cur_status.gPRA
            if target_pos == 0:
                print('minimum position (full opening)')
            elif target_pos == 255:
                print('maximum position (full closing)')
            else:
                print('gripper target position is {}'.format(target_pos))
        else:
            print('Individual mode is ON')
        return self.cur_status.gPRA

    def get_finger_a_target_pos(self):
        if self.outputMsg.rICF:
            target_pos = self.cur_status.gPRA
            if target_pos == 0:
                print('minimum position (full opening)')
            elif target_pos == 255:
                print('maximum position (full closing)')
            else:
                print('Finger A target position is {}'.format(target_pos))
        else:
            print('Individual mode is OFF')
        return self.cur_status.gPRA

    def get_gripper_current_pos(self):
        cur_pos = self.cur_status.gPOA
        if self.outputMsg.rICF == 0:
            if cur_pos == 0:
                print('minimum position (full opening)')
            elif cur_pos == 255:
                print('maximum position (full closing)')
            else:
                print('Gripper pos is {}'.format(cur_pos))
        else:
            print('Individual mode is ON')
        return self.cur_status.gPOA

    def get_finger_a_current_pos(self):
        cur_pos = self.cur_status.gPOA
        if cur_pos == 0:
            print('minimum position (full opening)')
        elif cur_pos == 255:
            print('maximum position (full closing)')
        else:
            print('Finger A pos is {}'.format(cur_pos))
        return self.cur_status.gPOA

    def get_finger_a_force(self):
        # TODO: Fix bugs. gCUA is not force. It is current consumption.
        force = self.cur_status.gCUA
        if force == 0:
            print('Finger A is weak')
        elif force == 255:
            print('Finger A is at Full force')
        else:
            print('Finger A is exerting {} amount of force'.format(force))
        return self.cur_status.gCUA

    def get_finger_b_target_pos(self):
        tar_pos = self.cur_status.gPRB
        if tar_pos == 0:
            print('minimum position (full opening)')
        elif tar_pos == 255:
            print('maximum position (full closing)')
        else:
            print('Finger B target position is {}'.format(tar_pos))
        return self.cur_status.gPRB

    def get_finger_b_current_pos(self):
        cur_pos = self.cur_status.gPOB
        if cur_pos == 0:
            print('minimum position (full opening)')
        elif cur_pos == 255:
            print('maximum position (full closing)')
        else:
            print('Finger B pos is {}'.format(cur_pos))
        return self.cur_status.gPOB

    def get_finger_b_force(self):
        force = self.cur_status.gCUB
        if force == 0:
            print('Finger B is weak')
        elif force == 255:
            print('Finger B is at Full force')
        else:
            print('Finger B is exerting {} amount of force'.format(force))
        return self.cur_status.gCUB

    def get_finger_c_target_pos(self):
        tar_pos = self.cur_status.gPRC
        if tar_pos == 0:
            print('minimum position (full opening)')
        elif tar_pos == 255:
            print('maximum position (full closing)')
        else:
            print('Finger C target position is {}'.format(tar_pos))
        return self.cur_status.gPRC

    def get_finger_c_current_pos(self):
        cur_pos = self.cur_status.gPOC
        if cur_pos == 0:
            print('minimum position (full opening)')
        elif cur_pos == 255:
            print('maximum position (full closing)')
        else:
            print('Finger C pos is {}'.format(cur_pos))
        return self.cur_status.gPOC

    def get_finger_c_force(self):
        force = self.cur_status.gCUC
        if force == 0:
            print('Finger C is weak')
        elif force == 255:
            print('Finger C is at Full force')
        else:
            print('Finger C is exerting {} amount of force'.format(force))
        return self.cur_status.gCUC

    def get_scissor_target_pos(self):
        tar_pos = self.cur_status.gPRS
        if tar_pos == 0:
            print('minimum position (full opening)')
        elif tar_pos == 255:
            print('maximum position (full closing)')
        else:
            print('scissor target position is {}'.format(tar_pos))
        return self.cur_status.gPRS

    def get_scissor_current_pos(self):
        cur_pos = self.cur_status.gPOS
        if cur_pos == 0:
            print('minimum position (full opening)')
        elif cur_pos == 255:
            print('maximum position (full closing)')
        else:
            print('Finger C pos is {}'.format(cur_pos))
        return self.cur_status.gPOS

    def get_scissor_target_force(self):
        force = self.cur_status.gCUS
        if force == 0:
            print('Finger C is weak')
        elif force == 255:
            print('Finger C is at Full force')
        else:
            print('Finger C is exerting {} amount of force'.format(force))
        return self.cur_status.gCUS

    ############################################################################################################

    ############################################     function         ##########################################
    def is_stopped(self):
        if  self.cur_status.gSTA and \
            self.cur_status.gDTA and\
            self.cur_status.gDTB and\
            self.cur_status.gDTC:
            return True
        else:
            # print ('GTO')
            # print (self.cur_status.gGTO)
            # print ('STA')
            # print (self.cur_status.gSTA)
            # print ('DTA')
            # print(self.cur_status.gDTA)
            # print ('DTB')
            # print(self.cur_status.gDTB)
            # print ('DTC')
            # print(self.cur_status.gDTC)
            # print ('DTS')
            # print(self.cur_status.gDTS)
            return False

    def is_move(self):
        if self.cur_status.gSTA == 0 or self.cur_status.gDTA == 0 or\
            self.cur_status.gDTB == 0 or self.cur_status.gDTC:
            return True
        else:
            return False

    def is_done(self):
        # target pos = rPRA , rPRB , rPRC
        # cur pos = gPOA , gPOB , gPOC
        while not rospy.is_shutdown():
            if self.outputMsg.rPRA == self.cur_status.gPOA and\
               self.outputMsg.rPRB == self.cur_status.gPOB and\
               self.outputMsg.rPRC == self.cur_status.gPOC:

                print('reached target value')
                return True

            else:
                if self.is_stopped():
                    # self.get_motion_status()
                    # self.get_finger_a_status()
                    # self.get_finger_b_status()
                    # self.get_finger_c_status()
                    return True

    def wait_for_connection(self, timeout =-1):
        rospy.sleep(0.1)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if timeout >= 0.0 and rospy.get_time() - start_time > timeout:
                return False
            if self.cur_status is not None:
                return True
        return False

    def wait_until_stopped(self, timeout=-1):
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if (timeout >= 0.0 and rospy.get_time() - start_time > timeout) or self.is_move:
                return False
            if self.is_stopped():
                # print('ooooo')
                return True
        return False

    def wait_until_moving(self, timeout=-1):
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if (timeout >= 0.0 and rospy.get_time() - start_time > timeout) or self.is_reset():
                return False
            if not self.is_move():
                return True
        return False

    def reset(self):
        cmd = outputMsg()
        cmd.rACT = 0
        self.cmd_pub.publish(cmd)

    def activate(self):
        self.active()
        self.go_to_request()
        self.set_grasping_speed(255)
        self.set_grasping_force(150)

    def close(self):
        self.outputMsg.rMOD = self.get_mode()
        self.set_target_pos(255)
        self.msg_pub()

    def open(self):
        self.outputMsg.rMOD = self.get_mode()
        self.set_target_pos(0)
        self.msg_pub()

    def faster(self):
        self.outputMsg.rSPA += 25
        if self.outputMsg.rSPA > 255:
            self.outputMsg.rSPA = 255

    def slower(self):
        self.outputMsg.rSPA -= 25
        if self.outputMsg.rSPA < 0:
            self.outputMsg.rSPA = 0

    def increase_force(self):
        self.outputMsg.rFRA += 25
        if self.outputMsg.rFRA > 255:
            self.outputMsg.rFRA = 255

    def decrease_force(self):
        self.outputMsg.rFRA -= 25
        if self.outputMsg.rFRA > 255:
            self.outputMsg.rFRA = 255

    def set_grasping_target_pos(self,target_pos,timeout =-1):
        self.set_target_pos(target_pos)
        self.outputMsg.rMOD = self.get_mode()
        # print (self.outputMsg.rMOD)
        self.msg_pub()

        # start_time = rospy.get_time()
        # while not rospy.is_shutdown():
        #     if timeout >= 0.0 and rospy.get_time() - start_time > timeout:
        #         return False
        #     if self.is_ready():
        #         return True
        # return False

    def msg_pub(self):
        self.active()
        self.go_to_request()
        self.cmd_pub.publish(self.outputMsg)
        rospy.sleep(0.2)


def main():
    rospy.init_node("robotiq_3f_gripper_ctrl_node")
    gripper = Robotiq3FGripperCTRL()
    gripper.reset()
    if gripper.wait_for_connection():
        print('gripper connection')
        if gripper.is_reset():
            print('reset state')
            gripper.activate()
            gripper.cmd_pub.publish(gripper.outputMsg)
            gripper.wait_until_stopped()

        if gripper.is_ready():
            gripper.activate()
            # order -> msg_pub -> is_done
            # open, close, set_grasping_targetpos
            # pinch mode close = 112
            # pinch mode open = 6
            print('ready state')
            # gripper.scissor_mode()

            gripper.pinch_mode()
            gripper.msg_pub()
            gripper.open()
            # gripper.is_done()
            # gripper.close()
            # gripper.set_grasping_target_pos()
            # gripper.is_done()
            # gripper.basic_mode()
            # gripper.msg_pub()

            # print("==============")
            # gripper.get_gripper_current_pos()
            # gripper.get_gripper_target_pos()
            # print("==============")
            # gripper.get_motion_status()
            # gripper.get_finger_a_status()
            # gripper.get_finger_b_status()
            # gripper.get_finger_c_status()

if __name__ == "__main__":
    main()
