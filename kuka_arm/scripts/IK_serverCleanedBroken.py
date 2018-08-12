#!/usr/bin/env python
# Copyright (C) 2017 Electric Movement Inc.
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
# All Rights Reserved.
# Author: Harsh Pandya
# Student Mods: 2018-08 CLeith
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

g_debugPrint = False

#===================================
def TFContainerFromFile(tfFileName):
    print("Loading tfFile " + tfFileName)
    tfFileIn = open(tfFileName, "rb" )
    tfNew = pickle.load(tfFileIn)
    tfFileIn.close()
    print("Finished.")
    return(tfNew)

class CTransformContainer(object):
    """
    Used to serialize Transforms to file.
    This is for speed optimization until I found that simplify()
    of the matrices wasn't really needed in practice
    """
    def __init__(self):
        pass

    def SaveToFile(self, tfFileName):
        print("saving tfFile " + tfFileName)
        tfFileOut = open(tfFileName, "wb" )
        pickle.dump(self, tfFileOut, pickle.HIGHEST_PROTOCOL)
        tfFileOut.close()

#=================================== Debug utilties
# Called from notebook only
def FKCalcTFfromJointAnglesArr(rawJointAngles):
    dictJointAngles = {dh.theta1: rawJointAngles[0], dh.theta2: rawJointAngles[1], dh.theta3: rawJointAngles[2], dh.theta4: rawJointAngles[3], dh.theta5: rawJointAngles[4], dh.theta6: rawJointAngles[5] }
    tfFK4D = FKCalcTFfromJointAnglesDict(dictJointAngles)
    return(tfFK4D)

def FKCalcTFfromJointAnglesDict(dictJointAngles):
    tfFK4D = g_tf.T0_GFinal.evalf(subs=dictJointAngles)
    return(tfFK4D)

def FKCalcPosefromJointAnglesArr(rawJointAngles):
    tfFK4D = FKCalcTFfromJointAnglesArr(rawJointAngles)
    poseCalc = TfToPose(tfFK4D)
    return(poseCalc)

def TfToPose(tfIn):
    poseNew = Pose()
    poseNew.position.x = tfIn[0, 3]
    poseNew.position.y = tfIn[1, 3]
    poseNew.position.z = tfIn[2, 3]
    tfInNPArr = numpy.array(tfIn)
    quat = tf.transformations.quaternion_from_matrix(tfInNPArr)
    poseNew.orientation.x = quat[0]
    poseNew.orientation.y = quat[1]
    poseNew.orientation.z = quat[2]
    poseNew.orientation.w = quat[3]
    return(poseNew)

#===================================
def CreateDHTransform(alpha, a, d, theta):
    """
    Create standard DH homogeneous transform for given DH parameters
    for Joint i.

    All the following arguments are sympy symbol objects or numbers
    :param alpha: Twist angle(i-1) from Zi-1 to Zi about Xi-1. Radians
    :param     a: Link length(i-1) from Zi-1 to Zi along Xi-1 where Xi-1 perp to both Z
    :param     d: Link offset(i) signed offset from Oi-1 to Oi along Zi. (Var for prismatic)
    :param theta: Joint angle theta(i)  from Xi-1 to Xi around Zi (Var for revolute). Radians
    :return: The DH transform. May still have symbols that would need to be resolved before use.
    """

    transform = Matrix(
        [[  cos(theta),                 -sin(theta),            0,            a],
         [  sin(theta)*cos(alpha),      cos(theta)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
         [  sin(theta)*sin(alpha),      cos(theta)*sin(alpha),  cos(alpha),   cos(alpha)*d],
         [  0,                          0,                      0,            1]])

    return transform

#===================================
def CreateTFFromLinear(tf3dIn):
    """
    Create an equivalent 4D Homegeneous transform from a 3D linear transform
    :param tf3dIn: The 3d input matrix
    :return: The 4D output matrix
    """
    transform = tf3dIn.row_join(Matrix([[0], [0], [0]]))
    transform = transform.col_join(Matrix([[0, 0, 0, 1]]))
    return transform

def CreateRotTransformX(symbolAngleRad):
    transform = Matrix([
        [1,      0,                     0],
        [0,      cos(symbolAngleRad),   -sin(symbolAngleRad)],
        [0,      sin(symbolAngleRad),   cos(symbolAngleRad)]])
    return transform

def CreateRotTransformY(symbolAngleRad):
    transform = Matrix([
        [cos(symbolAngleRad),   0,      sin(symbolAngleRad)],
        [0,                     1,      0],
        [-sin(symbolAngleRad),  0,      cos(symbolAngleRad)]])
    return transform

def CreateRotTransformZ(symbolAngleRad):
    transform =  Matrix([
        [cos(symbolAngleRad),   -sin(symbolAngleRad),   0],
        [sin(symbolAngleRad),   cos(symbolAngleRad),    0],
        [0,                     0,                      1]])
    return transform

def EulerFromPose(poseIn):
    poseReqQuat = [poseIn.orientation.x, poseIn.orientation.y, poseIn.orientation.z, poseIn.orientation.w,]
    curRoll, curPitch, curYaw = tf.transformations.euler_from_quaternion(poseReqQuat)
    return(curRoll, curPitch, curYaw)

################################ globals #############################

# Create DH sympy symbols
dh = type("dhSyms", (object,), {})
dh.theta1, dh.theta2, dh.theta3, dh.theta4, dh.theta5, dh.theta6, dh.theta7 = symbols('dh.theta1:8')
dh.d1, dh.d2, dh.d3, dh.d4, dh.d5, dh.d6, dh.d7 = symbols('d1:8')
dh.a0, dh.a1, dh.a2, dh.a3, dh.a4, dh.a5, dh.a6 = symbols('a0:7')
dh.alpha0, dh.alpha1, dh.alpha2, dh.alpha3, dh.alpha4, dh.alpha5, dh.alpha6 = symbols('dh.alpha0:7')

sym_r, sym_p, sym_y = symbols('sym_r sym_p sym_y') # roll,pitch,yaw for Gripper fix

# Create Modified DH parameters
g_dictDHVals = {
    dh.alpha0:     0,      dh.a0:     0,      dh.d1: 0.75,       #dh.theta1:
    dh.alpha1: -pi/2,      dh.a1:  0.35,      dh.d2: 0,          dh.theta2:     dh.theta2-pi/2,
    dh.alpha2:     0,      dh.a2:  1.25,      dh.d3: 0,          #dh.theta3:
    dh.alpha3: -pi/2,      dh.a3:-0.054,      dh.d4: 1.5,        #dh.theta4:
    dh.alpha4:  pi/2,      dh.a4:     0,      dh.d5: 0,          #dh.theta5:
    dh.alpha5: -pi/2,      dh.a5:     0,      dh.d6: 0,          #dh.theta6:
    dh.alpha6:     0,      dh.a6:     0,      dh.d7: 0.303,      dh.theta7:     0,
}

#=================================== Retrieve or Create Transforms
doLoadFromFile = False
dirpath = os.path.dirname(os.path.realpath(__file__))
g_tfContainerFileName = dirpath + "/TransformContainer.pypickle"

if (doLoadFromFile):
    g_tf = TFContainerFromFile(g_tfContainerFileName)
else:
    g_tf = CTransformContainer()

    print("Started individual transform processing...")
    g_tf.T0_1 = CreateDHTransform(dh.alpha0, dh.a0, dh.d1, dh.theta1).subs(g_dictDHVals)
    g_tf.T1_2 = CreateDHTransform(dh.alpha1, dh.a1, dh.d2, dh.theta2).subs(g_dictDHVals)
    g_tf.T2_3 = CreateDHTransform(dh.alpha2, dh.a2, dh.d3, dh.theta3).subs(g_dictDHVals)
    g_tf.T3_4 = CreateDHTransform(dh.alpha3, dh.a3, dh.d4, dh.theta4).subs(g_dictDHVals)
    g_tf.T4_5 = CreateDHTransform(dh.alpha4, dh.a4, dh.d5, dh.theta5).subs(g_dictDHVals)
    g_tf.T5_6 = CreateDHTransform(dh.alpha5, dh.a5, dh.d6, dh.theta6).subs(g_dictDHVals)
    g_tf.T6_G = CreateDHTransform(dh.alpha6, dh.a6, dh.d7, dh.theta7).subs(g_dictDHVals)
    print("Finished.")

    g_tf.R_corr = CreateRotTransformZ(pi) * CreateRotTransformY(-pi/2)
    g_tf.R_rpy = CreateRotTransformZ(sym_y) * CreateRotTransformY(sym_p) * CreateRotTransformX(sym_r)

    g_tf.T0_3  = (g_tf.T0_1  * g_tf.T1_2 * g_tf.T2_3)

    #T0_G  = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)
    g_tf.T0_G  = g_tf.T0_1 * g_tf.T1_2 * g_tf.T2_3 * g_tf.T3_4 * g_tf.T4_5 * g_tf.T5_6 * g_tf.T6_G
    g_tf.T_corr = CreateTFFromLinear(g_tf.R_corr)
    g_tf.T0_GFinal  = g_tf.T0_G * g_tf.T_corr

    g_tf.SaveToFile(g_tfContainerFileName)

#===================================
#=================================== IKCalculateJointAnglesFromPose()
#===================================
def IKCalculateJointAnglesFromPose(poseReq, tfs, dh, dictDHVals):
    """
    Given a single Robot pose returns the Joint angles needed
    to achieve that pose
    :param poseReq: The input pose to convert to joint angles
    :param tfs:
    :param dh: A container of sympy symbols of the DH parameters
    :param dictDHVals: A dictionary mapping DHSymbols to their values
    :return jointAnglesCalcRad: List of the calculated joint angles in radians
    :return  wristCenterCalcList: List of WristCenter XYZ values. Used for debug only
    """
    # Create wieldy local vars
    a1val = dictDHVals[dh.a1]
    a2val = dictDHVals[dh.a2]
    a3val = dictDHVals[dh.a3]
    d1val = dictDHVals[dh.d1]
    d4val = dictDHVals[dh.d4]
    d7val = dictDHVals[dh.d7]

    roll, pitch, yaw = EulerFromPose(poseReq)
    px, py, pz = poseReq.position.x, poseReq.position.y, poseReq.position.z
    grippos = Matrix([[px], [py], [pz]])

    R_rpysubs = tfs.R_rpy.evalf(subs={sym_r: roll, sym_p: pitch, sym_y: yaw})
    R0_6 = R_rpysubs * tfs.R_corr
    WC = grippos - d7val * R0_6[:,2] # d7 = .303

    if (False):
        print("@@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")
        print("rpy", roll, pitch, yaw)
        print("tfs.Rrpy", tfs.R_rpy)
        print("Rrpysub", R_rpysubs)
        print("tfs.R_corr", tfs.R_corr)
        print("WC", WC)
        print("R0_6", R0_6)


    rWC = sqrt(WC[0]**2 + WC[1]**2) - a1val # a1 = 0.35
    S = WC[2] - d1val # d1 = 0.75
    if (False):
        print("@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")

    sA = sqrt(d4val**2 + a3val**2) # = 1.501
    sB = sqrt(rWC**2 + S**2) # distance between WC  and joint2
    sC = a2val # 1.25

    # Triangle angles for sA, sB, sC
    thetaA = acos((sB**2 + sC**2 - sA**2)/(2*sB*sC))
    thetaB = acos((sA**2 + sC**2 - sB**2)/(2*sA*sC))
    thetaC = acos((sB**2 + sA**2 - sC**2)/(2*sA*sB))

    # Compute Robot Arm Joint Angles
    thetaJ1 = atan2(WC[1], WC[0])
    thetaJ2 = pi/2 - thetaA - atan2(S, rWC)
    thetaJ3 = pi/2 - thetaB - atan2(-a3val, d4val)
    if (False):
        print("@@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")
        print("rWC", rWC)
        print("S", S)
        print("sideABC", sA, sB, sC)
        print("AngleABC ", degrees(thetaA), degrees(thetaB), degrees(thetaC), )

    dictJointAnglesCalcRad = {dh.theta1: thetaJ1, dh.theta2: thetaJ2, dh.theta3: thetaJ3}

    R0_3raw = tfs.T0_1[0:3,0:3] * tfs.T1_2[0:3,0:3] * tfs.T2_3[0:3,0:3]
    R0_3 = R0_3raw.evalf(subs = dictJointAnglesCalcRad)
    R3_6 = R0_3.inv("ADJ") * R0_6 # "LU" or "ADJ"
    if (False):
        print("@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")
        print("R0_3", R0_3)
        print("R0_3raw", R0_3raw)
        print("R3_6 ", R3_6)

    # Create wieldy local vars
    r12 = R3_6[0,1]
    r13 = R3_6[0,2]
    r21 = R3_6[1,0]
    r22 = R3_6[1,1]
    r23 = R3_6[1,2]
    r32 = R3_6[2,1]
    r33 = R3_6[2,2]

    # Compute Robot Wrist Joint angles
    thetaJ4 = atan2(r33, -r13)
    thetaJ5 = atan2(sqrt(r13**2 + r33**2), r23)
    thetaJ6 = atan2(-r22, r21)

    # This joint angle return value gets packed in ROS trajectory struct by caller
    jointAnglesCalcRad = [thetaJ1, thetaJ2, thetaJ3, thetaJ4, thetaJ5, thetaJ6]
    if (False):
        print("@@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")
        print("JACalc ", jointAnglesCalcRad)

    # This flattened WC list is only used by the debug Notebook
    wristCenterCalcList = [WC[0], WC[1], WC[2]]

    return(wristCenterCalcList, jointAnglesCalcRad)

#=================================== ProcessMain()
def ProcessMain(request):
    trajectoryOut = []

    pp.pprint(request)
    for poseReq in request.poses:
        wristCenterCalcList, jointAnglesCalcRad = IKCalculateJointAnglesFromPose(poseReq, g_tf, dh, g_dictDHVals)

        # Package Joint angles into ROS Trajectory point struct
        jointTrajPoint = JointTrajectoryPoint()
        jointTrajPoint.positions = jointAnglesCalcRad
        # Add it to the list of points
        trajectoryOut.append(jointTrajPoint)

    # Package Trajectory points into ROS respsonse struct
    # This is an opaque, unexplained function!!!
    respIKTrajectory = CalculateIKResponse(trajectoryOut)
    return(respIKTrajectory)

#=================================== cbServiceRequestHandler()
def cbServiceRequestHandler(request):
    respIKTrajectory1 = None

    try:
        respIKTrajectory1 =  ProcessMain(request)

    except (rospy.ServiceException, rospy.ROSException), exc:
        excType, excMsg, excTraceback = sys.exc_info()
        print("Exception {} : {}".format(excType, excMsg))
        strTrace = traceback.format_tb(excTraceback, limit=2)
        print("TRACEBACK:")
        print(strTrace)
        print(" ")
        rospy.logerr("Service call failed: {}".format(exc))
    except:
        excType, excMsg, excTraceback = sys.exc_info()
        print("Exception {} : {}".format(excType, excMsg))
        strTrace = traceback.format_tb(excTraceback, limit=2)
        print("TRACEBACK:")
        print(strTrace)
    else:
        print "/calculate_ik IK Request finished..."

    return(respIKTrajectory1)

#=================================== ServerInit()
def ServerInit():
    filepath = os.path.realpath(__file__)
    print("Initializing IK_Server from file ", filepath)
    print("CWD: " + os.getcwd())

    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, cbServiceRequestHandler)
    print "IK_server ready on /calculate_ik service..."
    rospy.spin()
    print "IK_server shutting down..."

#=================================== main()
if __name__ == "__main__":
    ServerInit()
