"""
This script is based on the Robotiq3FGripperRobotInput.msg and Robotiq3FGripperRobotOutput.msg
from the robotiq_3f_gripper_articulated_msgs package.

Importing this script is similar to importing the fallowing line:
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotInput, Robotiq3FGripperRobotOutput

Please see the fallowing link for more information:
https://assets.robotiq.com/website-assets/support_documents/document/3-Finger_PDF_20190221.pdf
"""
from dataclasses import dataclass
from enum import IntEnum


class Robotiq3FMode(IntEnum):
    BASIC_MODE = 0
    PINCH_MODE = 1
    WIDE_MODE = 2
    SCISSOR_MODE = 3


class Robotiq3FFinger(IntEnum):
    ALL_FINGERS = 0
    FINGER_A = 1
    FINGER_B = 2
    FINGER_C = 3
    SCISSOR = 4


@dataclass
class Robotiq3FGripperRobotOutput:
    """Class defining the command that should be sent to a Robotiq 3F gripper."""

    # Gripper action request
    rACT: int = 0  # Activation request
    rMOD: int = 0  # Gripper mode (Use Robotiq3FMode)
    rGTO: int = 0  # "Go to" order (0: stop, 1: go)
    rATR: int = 0  # Automatic emergency release (0: off, 1: on)

    # Control strategy
    rGLV: int = 0  # Glove mode (0: off, 1: on)
    rICF: int = 0  # Individual control mode (0: off, 1: on)
    rICS: int = 0  # Individual controlled scissor axis (0: off, 1: on)

    # Finger A target (when individual control mode, otherwise all fingers)
    rPRA: int = 0  # Requested position (open: 0 ~ close: 255)
    rSPA: int = 0  # Opening or closing speed (22 mm/s: 0 ~ 110 mm/s: 255)
    rFRA: int = 0  # Force threshold of object detection (15 N: 0 ~ 60 N: 255)
    # Finger B target
    rPRB: int = 0
    rSPB: int = 0
    rFRB: int = 0
    # Finger C target
    rPRC: int = 0
    rSPC: int = 0
    rFRC: int = 0
    # Scissor axis target
    rPRS: int = 0
    rSPS: int = 0
    rFRS: int = 0

    def __repr__(self):
        return (
            "Robotiq3FGripperRobotOutput(\n"
            "  Activation, Mode, Go to, Automatic release\n"
            f"    rACT={self.rACT}, rMOD={self.rMOD}, rGTO={self.rGTO}, rATR={self.rATR},\n"
            "  Glove mode, Individual control mode, Individual controlled scissor axis\n"
            f"    rGLV={self.rGLV}, rICF={self.rICF}, rICS={self.rICS},\n"
            "  Finger A, B, C, Scissor axis: Position, Speed, Force\n"
            f"    rPRA={self.rPRA}, rSPA={self.rSPA}, rFRA={self.rFRA},\n"
            f"    rPRB={self.rPRB}, rSPB={self.rSPB}, rFRB={self.rFRB},\n"
            f"    rPRC={self.rPRC}, rSPC={self.rSPC}, rFRC={self.rFRC},\n"
            f"    rPRS={self.rPRS}, rSPS={self.rSPS}, rFRS={self.rFRS}\n"
            ")"
        )


@dataclass
class Robotiq3FGripperRobotInput:
    """Class defining the status of a Robotiq 3F gripper."""

    # Gripper status
    gACT: int = 0  # Activation status
    gMOD: int = 0  # Gripper mode (Use Robotiq3FMode)
    gGTO: int = 0  # Action status (0: stopped, 1: go)
    gIMC: int = 0  # Gripper status (0: in reset, 1: in activation, 2: in mode change, 3: activation & mode change done)
    gSTA: int = 0  # Motion status
    gDTA: int = 0  # Object detection status (finger A)
    gDTB: int = 0  # Object detection status (finger B)
    gDTC: int = 0  # Object detection status (finger C)
    gDTS: int = 0  # Object detection status (scissor axis)
    gFLT: int = 0  # Fault status

    # Finger A (when individual control mode, otherwise all fingers)
    gPRA: int = 0  # Requested position (open: 0 ~ close: 255)
    gPOA: int = 0  # Actual position (open: 0 ~ close: 255)
    gCUA: int = 0  # Instantaneous current consumption (0 ~ 255) => approximately 0.1 * gCUA (in mA)
    # Finger B
    gPRB: int = 0
    gPOB: int = 0
    gCUB: int = 0
    # Finger C
    gPRC: int = 0
    gPOC: int = 0
    gCUC: int = 0
    # Scissor axis
    gPRS: int = 0
    gPOS: int = 0
    gCUS: int = 0

    def __repr__(self):
        return (
            "Robotiq3FGripperRobotInput(\n"
            "  Activation, Mode, Action status\n"
            f"    gACT={self.gACT}, gMOD={self.gMOD}, gGTO={self.gGTO},\n"
            "  Gripper status (0: in reset, 1: in activation, 2: in mode change, 3: activation & mode change done)\n"
            f"    gIMC={self.gIMC},\n"
            "  gSTA : Motion status, returns the current motion of the Gripper fingers.\n"
            f"    gSTA={self.gSTA},\n"
            "  Object detection status (finger A, B, C, Scissor axis)\n"
            f"    gDTA={self.gDTA}, gDTB={self.gDTB}, gDTC={self.gDTC}, gDTS={self.gDTS},\n"
            "  Fault status (0: no fault, otherwise: some fault occurred)\n"
            f"    gFLT={self.gFLT},\n"
            "  Finger A, B, C, Scissor axis: Position, Actual position, Current consumption\n"
            f"    gPRA={self.gPRA}, gPOA={self.gPOA}, gCUA={self.gCUA},\n"
            f"    gPRB={self.gPRB}, gPOB={self.gPOB}, gCUB={self.gCUB},\n"
            f"    gPRC={self.gPRC}, gPOC={self.gPOC}, gCUC={self.gCUC},\n"
            f"    gPRS={self.gPRS}, gPOS={self.gPOS}, gCUS={self.gCUS}\n"
            ")"
        )
