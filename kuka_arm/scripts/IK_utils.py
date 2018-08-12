import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

import numpy
import os
import pickle
import collections
import types
import traceback
import pprint
pp = pprint.PrettyPrinter(indent=4)

#################################################
# This file contains utilities called from the
# jupyter notebook RosKinDebug.ipynb
# These are mainly for converting various types
# into other types that are easier for printing
# and utilities for printing out values
# in a compact format

g_debugPrint = False

#===================================
def PrintCompare(strHeader, calcVals, expectVals, strFormat="{:+9.4f}"):
    delta = numpy.subtract(expectVals, calcVals)

    if (False):
        print("@@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")
        print("calcVals", type(calcVals), calcVals)
        print("expectVals: ", type(expectVals), expectVals)
        print("deltaVals: ", type(delta), delta)

    print("========= {} calc/expect/err:".format(strHeader))
    print(map(strFormat.format, map(float, calcVals)))
    print(map(strFormat.format, map(float, expectVals)))
    print(map(strFormat.format, map(float, delta)))
    #print(map(strFormat.format, calcVals))
    #print(map(strFormat.format, expectVals))
    #print(map(strFormat.format,  delta))
    print("")

#=================================== Log()
def Log(obj):
    pp.pprint(obj)
    #print(obj)

#=================================== Spy()
def Spy(request, respIKTrajectory0, respIKTrajectory1):
    Log("Spy:")
    #Log("########## REQUEST: ###########")
    #Log(request)
    Log("################## jointTrajectory0 ###########")
    Log(respIKTrajectory0)
    Log("################## jointTrajectory1 ###########")
    Log(respIKTrajectory1)

def FKCalcAllPosesfromJointAnglesArr(rawJointAngles):
    print("Started FKCalcAllPose...")

    dictFKCalcAllPoses = collections.OrderedDict()

    dictJointAngles = {theta1: rawJointAngles[0], theta2: rawJointAngles[1], theta3: rawJointAngles[2], theta4: rawJointAngles[3], theta5: rawJointAngles[4], theta6: rawJointAngles[5] }
    dictFKCalcAllPoses['T0_1'] = TfToPose(T0_1.evalf(subs=dictJointAngles))
    dictFKCalcAllPoses['T0_2'] = TfToPose(T0_2.evalf(subs=dictJointAngles))
    dictFKCalcAllPoses['T0_3'] = TfToPose(T0_3.evalf(subs=dictJointAngles))
    dictFKCalcAllPoses['T0_4'] = TfToPose(T0_4.evalf(subs=dictJointAngles))
    dictFKCalcAllPoses['T0_5'] = TfToPose(T0_5.evalf(subs=dictJointAngles))
    dictFKCalcAllPoses['T0_6'] = TfToPose(T0_6.evalf(subs=dictJointAngles))
    dictFKCalcAllPoses['T0_G'] = TfToPose(T0_G.evalf(subs=dictJointAngles))
    dictFKCalcAllPoses['T0_GFinal'] = TfToPose(T0_GFinal.evalf(subs=dictJointAngles))
    print("Finished...")

    doPrint = True
    if (doPrint):
        for key, pose in dictFKCalcAllPoses.iteritems():
            print("=========FKCalc Pose for " + key)
            #pp.pprint(pose)
            print(pose)
            print("")

    return(dictFKCalcAllPoses)



#===================================
#===================================
def ExtractRawTestValues(testCase):
    reqPoseRaw = testCase[0]
    expectedWristCenter = testCase[1]
    expectedJointAngleRad = testCase[2]
    return(expectedJointAngleRad, expectedWristCenter, reqPoseRaw)

def ExtractRawTestValuesFromDict(dictRawPoses):
    reqPosesRaw = []
    expectedWristCenters = []
    expectedJointAnglesRad = []
    for key, testCase in dictRawPoses.iteritems():
        reqPosesRaw.append(testCase[0])
        expectedWristCenters.append(testCase[1])
        expectedJointAnglesRad.append(testCase[2])
    return(expectedJointAnglesRad, expectedWristCenters, reqPosesRaw)

def ConvertRawToPose(poseRaw):
    poseNew = Pose()
    poseNew.position.x = poseRaw[0][0]
    poseNew.position.y = poseRaw[0][1]
    poseNew.position.z = poseRaw[0][2]
    poseNew.orientation.x = poseRaw[1][0]
    poseNew.orientation.y = poseRaw[1][1]
    poseNew.orientation.z = poseRaw[1][2]
    poseNew.orientation.w = poseRaw[1][3]
    return(poseNew)

def ConvertPoseToRawPosition(poseIn):
    position = [None] * 3
    position[0] = poseIn.position.x
    position[1] = poseIn.position.y
    position[2] = poseIn.position.z
    return(position)

def ConvertPoseToRawQuaternion(poseIn):
    quaternion =  [None] * 4
    quaternion[0] = poseIn.orientation.x
    quaternion[1] = poseIn.orientation.y
    quaternion[2] = poseIn.orientation.z
    quaternion[3] = poseIn.orientation.w
    return(quaternion)

def ConvertPoseToRaw(poseIn):
    position = ConvertPoseToRawPosition(poseIn)
    quaternion = ConvertPoseToRawPosition(poseIn)
    return(quaternion, position)

def ConvertRawToPoses(posesRaw):
    posesNew = []
    for poseRaw in posesRaw:
        poseNew = ConvertRawPose(poseRaw)
        posesNew.append(poseNew)
    return(posesNew)
