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
import math

import os
import pickle
import collections
import types
import traceback
import pprint
pp = pprint.PrettyPrinter(indent=4)

g_debugPrint = False

################################ CHCode ##########################################

def TF_Matrix(q, alpha, a, d):
    T = Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                 0,           0,             1]])
    return(T)

def R_z(y):
    Rot_z = Matrix([[   cos(y),   -sin(y),         0],
                    [   sin(y),    cos(y),         0],
                    [        0,         0,         1]])
    return(Rot_z)

def R_y(p):
    Rot_y = Matrix([[   cos(p),         0,    sin(p)],
                    [        0,         1,         0],
                    [  -sin(p),         0,    cos(p)]])

    return(Rot_y)

def R_x(r):
    Rot_x = Matrix([[        1,         0,         0],
                    [        0,    cos(r),   -sin(r)],
                    [        0,    sin(r),    cos(r)]])

    return(Rot_x)

def TFContainerFromFile(tfFileName):
    tfFileIn = open(tfFileName, "rb" )
    tfNew = pickle.load(tfFileIn)
    tfFileIn.close()
    return(tfNew)

#===================================
class CTransformContainer(object):
    def __init__(self):
        pass

    def SaveToFile(self, tfFileName):
        tfFileOut = open(tfFileName, "wb" )
        pickle.dump(self, tfFileOut, pickle.HIGHEST_PROTOCOL)
        tfFileOut.close()


#=================================== Log()
def Log(obj):
    pp.pprint(obj)
    #print(obj)

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

#===================================
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

def FKCalcTFfromJointAnglesDict(dictJointAngles):
    tfFK4D = T0_GFinal.evalf(subs=dictJointAngles)
    return(tfFK4D)

def FKCalcTFfromJointAnglesArr(rawJointAngles):
    dictJointAngles = {theta1: rawJointAngles[0], theta2: rawJointAngles[1], theta3: rawJointAngles[2], theta4: rawJointAngles[3], theta5: rawJointAngles[4], theta6: rawJointAngles[5] }
    tfFK4D = FKCalcTFfromJointAnglesDict(dictJointAngles)
    return(tfFK4D)

def FKCalcPosefromJointAnglesArr(rawJointAngles):
    tfFK4D = FKCalcTFfromJointAnglesArr(rawJointAngles)
    poseCalc = TfToPose(tfFK4D)
    return(poseCalc)

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

def EulerFromPose(poseIn):
    poseReqQuat = [poseIn.orientation.x, poseIn.orientation.y, poseIn.orientation.z, poseIn.orientation.w,]
    curRoll, curPitch, curYaw = tf.transformations.euler_from_quaternion(poseReqQuat)
    return(curRoll, curPitch, curYaw)


#===================================
def CreateDHHomoTransform(alpha, a, d, theta):
    """
    Create standard DH homogeneous transform for given DH parameters
    for Joint i.

    :param dictSymbols: A dictionary of symbol name  <-> value pairs for substitution
    into the the sympy objects supplied as arguments
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
def CreateHomoTransformFromLinear(tf3dIn):
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

dh = type("dhSyms", (object,), {})
# Create FK sympy symbols
theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# Create Modified DH parameters
dictDHVals = {
    alpha0:     0,      a0:     0,      d1: 0.75,       #theta1:
    alpha1: -pi/2,      a1:  0.35,      d2: 0,          theta2:     theta2-pi/2,
    alpha2:     0,      a2:  1.25,      d3: 0,          #theta3:
    alpha3: -pi/2,      a3:-0.054,      d4: 1.5,        #theta4:
    alpha4:  pi/2,      a4:     0,      d5: 0,          #theta5:
    alpha5: -pi/2,      a5:     0,      d6: 0,          #theta6:
    alpha6:     0,      a6:     0,      d7: 0.303,      theta7:     0,
}

r, p, y = symbols('r p y') # roll,pitch,yaw for Gripper fix
R_corr = R_z(pi) * R_y(-pi/2)
R_rpy = R_z(y) * R_y(p) * R_x(r)

print("Started individual transform processing...")
T0_1 = CreateDHHomoTransform(alpha0, a0, d1, theta1).subs(dictDHVals)
T1_2 = CreateDHHomoTransform(alpha1, a1, d2, theta2).subs(dictDHVals)
T2_3 = CreateDHHomoTransform(alpha2, a2, d3, theta3).subs(dictDHVals)
T3_4 = CreateDHHomoTransform(alpha3, a3, d4, theta4).subs(dictDHVals)
T4_5 = CreateDHHomoTransform(alpha4, a4, d5, theta5).subs(dictDHVals)
T5_6 = CreateDHHomoTransform(alpha5, a5, d6, theta6).subs(dictDHVals)
T6_G = CreateDHHomoTransform(alpha6, a6, d7, theta7).subs(dictDHVals)
print("Finished.")

#print("Started composite transform processing... ")
T0_2  = (T0_1  * T1_2)
T0_3  = (T0_2  * T2_3)
#T0_2  = simplify(T0_1  * T1_2)
#T0_3  = simplify(T0_2  * T2_3)
#T0_4  = simplify(T0_3  * T3_4)
#T0_5  = simplify(T0_4  * T4_5)
#T0_6  = simplify(T0_5  * T5_6)
#T0_G  = simplify(T0_6  * T6_G)
#print("Finished.")

print("Started one line composite transform processing...")
#T0_G  = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)
T0_G  = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
print("Finished.")

print("Started gripper/EndEffector transform fixups, final FKtf T0_GFinal...")
tfZ180 = CreateRotTransformZ(math.pi)

tfYm90 = CreateRotTransformY(-math.pi/2.0)
#rotGripperOrientFix = simplify(tfZ180 * tfYm90)
rotGripperOrientFix = tfZ180 * tfYm90

#tfY180 = CreateRotTransformY(math.pi)
#rotGripperOrientFix = simplify(tfZ180 * tfY180)

tfGripperOrientFix = CreateHomoTransformFromLinear(rotGripperOrientFix)
#T0_GFinal = simplify(T0_G * tfGripperOrientFix)
T0_GFinal =T0_G * tfGripperOrientFix
#T0_GFinal = T0_GFinal.evalf(subs=dictDHVals)

symRoll, symPitch, symYaw = Symbol("symRoll"), Symbol("symPitch"), Symbol("symYaw")
symRot_X = CreateRotTransformX(symRoll)
symRot_Y = CreateRotTransformY(symPitch)
symRot_Z = CreateRotTransformZ(symYaw)
symRot_6_GUnfixed = symRot_Z * symRot_Y * symRot_X
symRot_6_G = symRot_6_GUnfixed  * rotGripperOrientFix
print("Finished.")

print("CWD " + os.getcwd())
dirpath = os.path.dirname(os.path.realpath(__file__))
g_tfContainerFileName = dirpath + "/TransformContainer.pypickle"

doLoadFromFile = False
if (doLoadFromFile):
    print("Loading tfFile " + g_tfContainerFileName)
    g_tf = TFContainerFromFile(g_tfContainerFileName)
    print("Finished.")
    T0_1 =  g_tf.T0_1
    T1_2 =  g_tf.T1_2
    T2_3 =  g_tf.T2_3
    T3_4 =  g_tf.T3_4
    T4_5 =  g_tf.T4_5
    T5_6 =  g_tf.T5_6
    T6_G =  g_tf.T6_G
    T0_2 =  g_tf.T0_2
    T0_3 =  g_tf.T0_3
    T0_GFinal = g_tf.T0_GFinal
else:
    g_tf = CTransformContainer()
    g_tf.T0_1 = T0_1
    g_tf.T1_2 = T1_2
    g_tf.T2_3 = T2_3
    g_tf.T3_4 = T3_4
    g_tf.T4_5 = T4_5
    g_tf.T5_6 = T5_6
    g_tf.T6_G = T6_G
    g_tf.T0_2 = T0_2
    g_tf.T0_3 = T0_3
    g_tf.T0_GFinal = T0_GFinal
    #g_tf.SaveToFile(g_tfContainerFileName)

#===================================
#=================================== IKCalculateJointAnglesFromPose()
#===================================
#def IKCalculateJointAnglesFromPose(poseReq, tfs, dictDictDHVals):
def IKCalculateJointAnglesFromPose(poseReq):
    global R_rpy

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [poseReq.orientation.x, poseReq.orientation.y, poseReq.orientation.z, poseReq.orientation.w])

    vecPoseReqPos = Matrix(ConvertPoseToRawPosition(poseReq))

    d7val = 0.303 # dictDHVals[d7]
    vOffset6_EE_CS6 = Matrix([[0.0], [0.0], [d7val]]) # Distance from WristCenter to EE along in CoordSys6... I think

    R_rpysubs = R_rpy.evalf(subs={r: roll, p: pitch, y: yaw})
    R0_6 = R_rpysubs * R_corr

    px, py, pz = poseReq.position.x, poseReq.position.y, poseReq.position.z
    EE = Matrix([[px], [py], [pz]])
    WC = EE - (0.303) * R0_6[:,2]

    if (True):
        print("@@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")
        print("rpy", roll, pitch, yaw)
        print("Rrpy", R_rpy)
        print("Rrpysub", R_rpysubs)
        print("R_corr", R_corr)

    if (True):
        print("@@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")
        print("WC", WC)
        print("R0_6", R0_6)

    thetaJ1 = atan2(WC[1], WC[0])

    rWC = sqrt(WC[0]**2 + WC[1]**2) - 0.35 # a1
    S = WC[2] - 0.75 # d1
    if (True):
        print("@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")
        print("r", rWC)
        print("S", S)


    sC = 1.25  # a2 = 1.25
    sA = 1.501 # sqrt(d4**2 + a3**2) = 1.501
    sB = sqrt(rWC**2 + S**2) #distance between WC at joint 5 and joint 2

    # now calculate theta a, b, c of the triangle, given sC, sB, and sA (cosin law)
    thetaA = acos((sB**2 + sC**2 - sA**2)/(2*sB*sC))
    thetaB = acos((sA**2 + sC**2 - sB**2)/(2*sA*sC))
    #thetaC = acos((sB**2 + sA**2 - sC**2)/(2*sA*sB))

    # now calculate theta 2 and theta 3
    #thetaJ2 = pi/2 - thetaa - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)
    #thetaJ3 = (pi/2-0.036) - thetab #1.5348 is the angle at the original position
    thetaJ2 = (pi/2) - thetaA - atan2(S, rWC)
    thetaJ3 = (pi/2) - thetaB - atan2(-a3, d4)

    thetaJ2 = thetaJ2.evalf(subs=dictDHVals)
    thetaJ3 = thetaJ3.evalf(subs=dictDHVals)

    dictJointAnglesCalcRad = {theta1: thetaJ1, theta2: thetaJ2, theta3: thetaJ3}

    # now calculate 4, 5, and 6
    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs = dictJointAnglesCalcRad)
    #R3_6 = R0_3.inv("LU")*ROT_EE
    R3_6 = R0_3.inv("ADJ")*R0_6

    #https://pdfs.semanticscholar.org/6681/37fa4b875d890f446e689eea1e334bcf6bf6.pdf
    #theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    #theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    #theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    # R3_6 Matrix Values
    r13 = R3_6[0,2]
    r33 = R3_6[2,2]
    r23 = R3_6[1,2]
    r21 = R3_6[1,0]
    r22 = R3_6[1,1]
    r12 = R3_6[0,1]
    r32 = R3_6[2,1]

    thetaJ4 = atan2(r33, -r13)
    thetaJ5 = atan2(sqrt(r13**2 + r33**2), r23)
    thetaJ6 = atan2(-r22, r21)

    if (False):
        print("@@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")
        print("thetaJ1 ", thetaJ1)
        print("thetaJ2 ", thetaJ2)
        print("thetaJ3 ", thetaJ3 )

    # ---------------------------------------------------------
    #side_a = 1.501
    #side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) -0.35), 2) + pow((WC[2] - 0.75), 2))
    #side_c = 1.25

    #angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
    #angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
    #angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

    if (False):
        print("@@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")
        print("SideB ", side_b)
        print("AngleABC ", degrees(angle_a), degrees(angle_b), degrees(angle_c), )

    #thetaJ2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0]  + WC[1] * WC[1]) - 0.35)
    #thetaJ3 = pi/2 - (angle_b + 0.036) # Account for sag in link4 of -0.054

    #dictJointAnglesCalcRad = {theta1: thetaJ1, theta2: thetaJ2, theta3: thetaJ3}

    #R0_3 = T0_1[0:3,0:3] * T0_2[0:3,0:3] * T0_3[0:3,0:3]
    #R0_3 = R0_3.evalf(subs=dictJointAnglesCalcRad)

    #R3_6 = R0_3.inv("LU") * curROT_EE

    #thetaJ4 = atan2(R3_6[2,2], -R3_6[0,2])
    #thetaJ5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    #thetaJ6 = atan2(-R3_6[1,1], R3_6[1,0])

    jointAnglesCalcRad = [thetaJ1, thetaJ2, thetaJ3, thetaJ4, thetaJ5, thetaJ6]
    wristCenterCalcList = [WC[0], WC[1], WC[2]]
    if (False):
        print("@@@@@@@@@@@@@@@@@@@@@@@@ debugPrint @@@@@@@@@@@@@@@@@@@@@@@@")
        print("WC ", WC)
        print("WCList ", wristCenterCalcList)
        print("JACalc ", jointAnglesCalcRad)

    return(wristCenterCalcList, jointAnglesCalcRad)

#=================================== ProcessMain()
def ProcessMain(request):
    trajectoryOut = []

    pp.pprint(request)
    for poseReq in request.poses:
        wristCenterCalcList, jointAnglesCalcRad = IKCalculateJointAnglesFromPose(poseReq)
        jointTrajPoint = JointTrajectoryPoint()
        jointTrajPoint.positions = jointAnglesCalcRad
        trajectoryOut.append(jointTrajPoint)

    respIKTrajectory = CalculateIKResponse(trajectoryOut)
    return(respIKTrajectory)

#=================================== cbServiceRequestHandler()
def cbServiceRequestHandler(request):
    respIKTrajectory = None

    try:
        respIKTrajectory =  ProcessMain(request)

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

    return(respIKTrajectory)

#=================================== ServerInit()
def ServerInit():
    filepath = os.path.realpath(__file__)
    print("Initializing IK_Server from file ", filepath)
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, cbServiceRequestHandler)
    print "IK_server ready on /calculate_ik service..."
    rospy.spin()
    print "IK_server shutting down..."

#=================================== main()
if __name__ == "__main__":
    ServerInit()
