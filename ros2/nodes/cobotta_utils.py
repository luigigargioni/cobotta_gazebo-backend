from ast import dump
from numpy import pi, absolute, array
from math import cos, sin, radians, ceil, sqrt, atan2, degrees, asin
from enum import Enum
from json import loads
import socket

from bcapclient import BCAPClient


PIX_MM_RATIO = 9.222
CAMERA_ROBOT_DISTANCE = 52.38570925983608  # mm

BCAP_MACHINE_NAME = "localhost"

class CaoParams(Enum):
    VRC = "CaoProv.DENSO.VRC"
    ENGINE = "CAO.CaoEngine"
    CANON_CAMERA = "CaoProv.Canon.N10-W02"
    RC8 = "CaoProv.DENSO.RC8"


class RobotAction(Enum):
    MOTOR = "Motor"
    GIVE_ARM = "GiveArm"
    TAKE_ARM = "TakeArm"
    ARM_0 = "Arm0"
    HAND_MOVE_A = "HandMoveA"
    HAND_MOVE_H = "HandMoveH"
    ROBOT_0 = "robot0"

INITIAL_POSITION = """@0 P(177.483268825558, -44.478627592948996, 254.99815172770593, -179.98842099994923, 0,
                                    179.99584205147127, 261.0)"""
CURRENT_POSITION = "@CURRENT_POSITION"
CURRENT_ANGLE = "@CURRENT_ANGLE"
DEFAULT_TIMEOUT = 14400
MAX_SPEED = "SPEED=100"
HALF_SPEED = "SPEED=50"
CALIBRATION_HEIGHT = "254.99815172770593"

def robot_take_arm(client, hRobot):
    client.robot_execute(hRobot, RobotAction.TAKE_ARM.value, [0, 1])


def robot_give_arm(client, hRobot):
    client.robot_execute(hRobot, RobotAction.GIVE_ARM.value)


def robot_give_arm_cao(caoRobot):
    caoRobot.Execute(RobotAction.GIVE_ARM.value)


def robot_motor(client, hRobot):
    client.robot_execute(hRobot, RobotAction.MOTOR.value, [1, 0])


def robot_motor_cao(caoRobot):
    caoRobot.Execute(RobotAction.MOTOR.value, [1, 0])


def connect(host, port, timeout, provider=CaoParams.VRC.value):
    client = BCAPClient(host, port, timeout)
    client.service_start("")
    Name = ""
    Provider = provider
    Machine = BCAP_MACHINE_NAME
    Option = ""
    hCtrl = client.controller_connect(Name, Provider, Machine, Option)
    hRobot = client.controller_getrobot(hCtrl, RobotAction.ARM_0.value)
    # robot_give_arm(client, hRobot)
    # robot_take_arm(client, hRobot)
    # robot_motor(client, hRobot)
    return (client, hCtrl, hRobot)


def disconnect(client, hCtrl, hRobot):
    robot_motor(client, hRobot)
    robot_give_arm(client, hRobot)
    client.controller_disconnect(hCtrl)
    client.service_stop()


def robot_getvar(client, hRobot, name):
    assert isinstance(client, BCAPClient)
    var_handle = client.robot_getvariable(hRobot, name)
    value = client.variable_getvalue(var_handle)
    client.variable_release(var_handle)
    return value

def switch_bcap_to_orin(client, hRobot, caoRobot):
    robot_give_arm(client, hRobot)
    robot_take_arm(client, hRobot)
    robot_motor_cao(caoRobot)


def switch_orin_to_bcap(client, hRobot, caoRobot):
    robot_give_arm_cao(caoRobot)
    robot_take_arm(client, hRobot)
    robot_motor(client, hRobot)


def list_to_string_position(pos):
    return "P(" + ", ".join(str(i) for i in pos) + ")"


def list_to_string_joints(pos):
    return "J(" + ", ".join(str(i) for i in pos) + ")"


def move_to_new_pos(client, hRobot, new_x, new_y, mode=2):
    curr_pos = robot_getvar(client, hRobot, CURRENT_POSITION)
    curr_pos[0] = new_x
    curr_pos[1] = new_y
    client.robot_move(hRobot, mode, list_to_string_position(curr_pos), MAX_SPEED)

def move_to_angle(client, hRobot, positions):

    # Command = "TakeArm"
    # Param = [0,0]
    # client.robot_execute(hRobot,Command,Param)
    # print("TakeArm")
    
    # ### Motor On
    # Command = "Motor"
    # Param = [1,0]
    # client.robot_execute(hRobot,Command,Param)
    # print("Motor On")

    # ### Move Initialize Position
    # Comp=1
    # Pose = [positions,"J","@E"]
    # client.robot_move(hRobot,Comp,Pose,MAX_SPEED)
    # print("Complete Move P,@E ", list_to_string_joints(positions))

    # robot_give_arm(client, hRobot)
    # print("GiveArm")

    j1 = positions[0]
    j2 = positions[1]
    j3 = positions[2]
    j4 = positions[3]
    j5 = positions[4]
    j6 = positions[5]
    client.robot_execute(hRobot, "TakeArm")
    client.robot_execute(hRobot, "Motor", [1, 0])
    client.robot_execute(hRobot, "ExtSpeed", 70)
    client.robot_move(
        hRobot, 1, "@P J({},{},{},{},{},{})".format(j1, j2, j3, j4, j5, j6)
    )
    client.robot_execute(hRobot, "GiveArm")

def get_position(client, hRobot):
    curr_pos = robot_getvar(client, hRobot, CURRENT_POSITION)
    position = {
        "X": +curr_pos[0],
        "Y": +curr_pos[1],
        "Z": +curr_pos[2],
        "RX": +curr_pos[3],
        "RY": +curr_pos[4],
        "RZ": +curr_pos[5],
        "FIG": +curr_pos[6],
    }
    response = {"position": position}

    return response

def get_angle_joints(client, hRobot):
    return robot_getvar(client, hRobot, CURRENT_ANGLE)


def move_to_calibration_position(client: BCAPClient, hRobot):
    client.robot_execute(hRobot, "TakeArm")
    client.robot_execute(hRobot, "Motor", [1, 0])
    client.robot_execute(hRobot, "ExtSpeed", 70)
    client.robot_move(
        hRobot,
        1,
        INITIAL_POSITION
    )
    client.robot_execute(hRobot, "GiveArm")


def open_hand(client: BCAPClient, hRobot: any, caoRobot: any, ctrl: any):
    switch_bcap_to_orin(client, hRobot, caoRobot)
    # Open hand to release object. HandMoveA (open in mm, speed)
    ctrl.Execute(RobotAction.HAND_MOVE_A.value, [30, 25])
    switch_orin_to_bcap(client, hRobot, caoRobot)
