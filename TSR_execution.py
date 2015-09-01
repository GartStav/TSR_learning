#!/usr/bin/env python
'''
TSR_execution.py
Takes as input TSR description file and performs the motion. Depending on the motion the environment need to be adjusted.
For the code that performs motion on real-world (Vicon) data look at the "heres_how" project.

Author: Artem Gritsenko
Worcester Polytechnic Institute, ArcLab
July 2015
'''
import time
import csv
import sys
import os
import openravepy
#import numpy as np
from changepoint.mean_shift_model import MeanShiftModel
import scipy
from scipy import signal
import pickle
import shutil
from matplotlib.mlab import PCA as mlabPCA
from copy import deepcopy

# drawing
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch
from wpi_planning_utilities.TransformMatrix import *
from wpi_planning_utilities.rodrigues import *

from wpi_planning_utilities.TSR import *
from wpi_planning_utilities import *
from numpy import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *



class TSR_exec():

    def __init__(self, env_path):

        print "Setting up the environment..."
        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.Reset()
        self.env.Load(env_path)
        self.robot = self.env.GetRobots()[0]
        time.sleep(0.1)

        self.GenerateJointDict()
        self.SetInManipulationConfiguration()
        self.robot.SetActiveManipulator("rightarm")

        self.drawingHandles = []

        self.right_arm_idx = 5
        self.left_arm_idx = 7

        self.r_arm_indices = self.robot.GetManipulators()[self.right_arm_idx].GetArmIndices()
        self.l_arm_indices = self.robot.GetManipulators()[self.left_arm_idx].GetArmIndices()

        self.arm_indices = concatenate((self.r_arm_indices, self.l_arm_indices), axis=0)

        self.traj_id = 0

        traj_id = 0


    def waitrobot(self):
        q = self.robot.GetDOFValues()
        t = self.robot.GetTransform()
        self.robot.GetController().SetDesired(q, t)
        self.robot.WaitForController(0)


    def GenerateJointDict(self):
        self.jointDict = {}
        self.jointNames = {}
        for jIdx, j in enumerate(self.robot.GetJoints()):
            self.jointDict[j.GetName()] = jIdx
            self.jointNames[jIdx] = j.GetName()


    def SetInManipulationConfiguration(self):

        #self.robot.SetDOFValues([0.0], [self.jointDict["torso_lift_joint"]])
        self.robot.SetDOFValues([0.0], [self.jointDict["torso_lift_joint"]]) # Torso up

        self.robot.SetDOFValues([1.9], [self.jointDict["l_shoulder_pan_joint"]])
        self.robot.SetDOFValues([0.3], [self.jointDict["l_shoulder_lift_joint"]])
        self.robot.SetDOFValues([0.3], [self.jointDict["l_upper_arm_roll_joint"]])
        self.robot.SetDOFValues([-1.5], [self.jointDict["l_elbow_flex_joint"]])
        self.robot.SetDOFValues([0.0], [self.jointDict["l_forearm_roll_joint"]])

        self.robot.SetDOFValues([-0.2], [self.jointDict["l_wrist_flex_joint"]])
        self.robot.SetDOFValues([-0.2], [self.jointDict["l_wrist_roll_joint"]])

        self.robot.SetDOFValues([-1.9], [self.jointDict["r_shoulder_pan_joint"]])
        self.robot.SetDOFValues([0.3], [self.jointDict["r_shoulder_lift_joint"]])
        self.robot.SetDOFValues([-0.3], [self.jointDict["r_upper_arm_roll_joint"]])
        self.robot.SetDOFValues([-1.5], [self.jointDict["r_elbow_flex_joint"]])
        self.robot.SetDOFValues([0.0], [self.jointDict["r_forearm_roll_joint"]])

        self.robot.SetDOFValues([-0.2], [self.jointDict["r_wrist_flex_joint"]])
        self.robot.SetDOFValues([-0.2], [self.jointDict["r_wrist_roll_joint"]])

        self.robot.SetDOFValues([0.09], [self.jointDict["l_gripper_joint"]])
        self.robot.SetDOFValues([0.09], [self.jointDict["r_gripper_joint"]])

        self.robot.SetDOFValues([0.34], [self.jointDict["r_gripper_l_finger_joint"]])
        self.robot.SetDOFValues([0.34], [self.jointDict["l_gripper_l_finger_joint"]])

        self.robot.SetDOFValues([-1.16819152996], [self.jointDict["l_wrist_flex_joint"]])
        self.robot.SetDOFValues([-0.73930679618], [self.jointDict["l_wrist_roll_joint"]])
        self.robot.SetDOFValues([-1.16785645058], [self.jointDict["r_wrist_flex_joint"]])
        self.robot.SetDOFValues([0.738326004881], [self.jointDict["r_wrist_roll_joint"]])

        # print q[ self.jointDict["l_wrist_flex_joint"] ]
        # print q[ self.jointDict["l_wrist_roll_joint"] ]
        # print q[ self.jointDict["r_wrist_flex_joint"] ]
        # print q[ self.jointDict["r_wrist_roll_joint"] ]

        #self.pr2_planning.PrintJointLimits()

        self.q_resting = self.robot.GetDOFValues()

        return

    # Plans a trajectory using a CBiRRT problem and goaljoints (active dofs)
    def PlanTrajectory(probs_cbirrt, q_init, q_goal, TSRChainString, smoothingitrs,
                       error_code_str, mimicdof, psample, activedofs ):

        robot.GetController().Reset(0)

        if(type(q_goal) == type("")):
            goaljoints = deepcopy(str2num(q_goal))
        else:
            goaljoints = deepcopy(q_goal)

        robot.SetActiveDOFValues(q_init)
        q_tmp = robot.GetDOFValues()

        t = robot.GetTransform()
        robot.GetController().SetDesired(q_tmp, t)
        time.sleep(1)

        robot.SetActiveDOFs(activedofs)

        print robot.GetDOFValues()
        print goaljoints
        print robot.GetActiveDOFValues()
        #sys.stdin.readline()

        cmdStr = 'RunCBiRRT timelimit 40 '

        print "CALL CBIRRT"
        try:
            if psample != None:
                cmdStr += 'psample '+str(psample)+' '

            cmdStr += 'smoothingitrs '+str(smoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainString
            answer = '0'
            answer = probs_cbirrt.SendCommand(cmdStr)

            print "RunCBiRRT answer: ", str(answer)
            if answer[0] != '1':
                return [False, error_code_str+" - "+answer[1:]]
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e
            return [False, "CBiRRT Plug-in Exception."]

        return [True, ""]

    def DisplayTrajectory(self):

        print "displaying trajectory"

        if True:

            answers = []
            try:
                traj = RaveCreateTrajectory(self.env, '')
                f = open('cmovetraj.txt', 'r')
                shutil.copy2('cmovetraj.txt', 'trajs/cmovetraj' + str(self.traj_id).zfill(4) + '.txt')
                traj.deserialize(f.read())
                f.close()

                self.traj_id += 1

                # if do_screw_motion:
                #     [traj, q_end] = self.AddScrewMotion(traj)

                # self.robot.GetController().SetPath(traj)

                answers.append(self.probs_cbirrt.SendCommand('traj cmovetraj.txt'))
                print "traj call answer: ", str(answers[-1])

                self.robot.WaitForController(0)

                # if do_screw_motion:
                #     self.robot.SetDOFValues(q_end)

            except openrave_exception, e:
                print e
                return [False, "OS exception in PlayTrajectory."]

            self.robot.GetController().Reset(0)

            # print "Press return to exit."
            # sys.stdin.readline()

            q = self.robot.GetDOFValues()
            t = self.robot.GetTransform()
            self.robot.GetController().SetDesired(q, t)
            self.robot.WaitForController(0)


    def PlanSimpleArmMotion(env, robot, probs_cbirrt, q_goal, active_dofs, tsr_chain_string="", object=None):

        robot.SetActiveDOFs(active_dofs)
        q_cur = robot.GetActiveDOFValues()

        # Basic CbiRRT variables
        smoothingitrs = 30
        mimicdof = None
        error_code_str = ""
        psample = 0.02
        activedofs = active_dofs
        success = False

        with env:

            [success, info] = PlanTrajectory(probs_cbirrt, q_cur, q_goal, tsr_chain_string, smoothingitrs,
                                                               error_code_str, mimicdof, psample, activedofs)
            print "end planning"
            if success != 1:
                print "planning for simple arm motion failed"
                return [success, info]

        DisplayTrajectory(env, robot, probs_cbirrt)

        return [success, info]

    def PlanMotion(env, robot, manip_indicies, T0w_path, Twe_path, Bw_path, T0w_goal, Twe_goal, Bw_goal):

        print "Perform Reaching Action"

        arm_indicies = array([])
        if manip_indicies.count(5):
            arm_indicies = robot.GetManipulators()[5].GetArmIndices()
        if manip_indicies.count(7):
            arm_indicies = concatenate((arm_indicies, robot.GetManipulators()[7].GetArmIndices()), axis=0)

        print arm_indicies
        print manip_indicies[0]

        robot.SetActiveDOFs(arm_indicies)
        #q_init = self.robot.GetActiveDOFValues()



        print Bw_path

        TSRstring_path = SerializeTSR(manip_indicies[0], 'NULL', T0w_path, Twe_path, Bw_path)# mat([-1, .1, -0, 0.8, -0, 0.19, -1000, 1000, -1000, 1000, -1000, 1000])) #Bw_path)

        TSRstring_goal = SerializeTSR(manip_indicies[0], 'NULL', T0w_goal, Twe_goal, mat([-0, 0, -0, 0, -0, 0, -0, 0, -0, 0, -0, 0])) #Bw_goal)

        # pack the TSRs into a string
        TSRChainString = '%s %s' % (SerializeTSRChain(0, 0, 1, 1, TSRstring_path, 'NULL', []),
                                    SerializeTSRChain(0, 1, 0, 1, TSRstring_goal, 'NULL', []))

        del drawingHandles[:]

        drawingHandles.append(misc.DrawAxes(env, T0w_path, 0.3))
        drawingHandles.append(misc.DrawAxes(env, T0w_goal, 0.3))
        # drawingHandles.append(misc.DrawAxes(env, T0w_goal, 0.3))
        # drawingHandles.append(misc.DrawAxes(env, Twe_goal, 0.3))

        print "press enter to plan"
        sys.stdin.readline()

        [success, info] = PlanSimpleArmMotion(env, robot, probs_cbirrt, [], arm_indicies, TSRChainString)


    def SetProblems(self):
        self.probs_cbirrt = RaveCreateModule(self.env, 'CBiRRT')
        try:
            self.env.AddModule(self.probs_cbirrt, self.robot.GetName())
        except openrave_exception, e:
            print e
        print "Getting Loaded Problems"


    def GetObjectByName(self, obj_name):
        obj = None
        for body in self.env.GetBodies():
            if body.GetName() == obj_name:
                obj = body
                break
        if obj == None:
            print "Warning: Object " + obj_name + " does not exist"
            return None
        return obj

    def GetRightHandTransform(self):
        return self.robot.GetManipulators()[self.right_arm_idx].GetEndEffectorTransform()

    def GetLeftHandTransform(self):
        return self.robot.GetManipulators()[self.left_arm_idx].GetEndEffectorTransform()

    def trans_to_str(self, T):
        myStr = ""
        for c in range(0,3):
            for r in range(0,3):
                myStr += str(T[r,c])+" "

        for r in range(0,3):
            myStr += str(T[r,3])+" "

        #print "Tee string : "
        #print myStr
        return myStr

    def FindTwoArmsIK(self, T0_RH, T0_LH, open_hands, quiet=False):
        arg1 = self.trans_to_str(T0_LH)
        arg2 = self.trans_to_str(T0_RH)
        q_ik = self.probs_cbirrt.SendCommand('DoGeneralIK exec nummanips 2 maniptm 7 '+arg1+' maniptm 5 '+arg2)
        if q_ik == '' or self.robot.CheckSelfCollision() or self.env.CheckCollision(self.robot):
            if not quiet:
                if q_ik != '':
                    # q = self.robot.GetDOFValues()
                    # t = self.robot.GetTransform()
                    # self.robot.GetController().SetDesired(q, t)
                    # self.robot.WaitForController(0)
                    # sys.stdin.readline()
                    report = CollisionReport()
                    in_col = self.robot.CheckSelfCollision(report)
                    print "in col : ", in_col
                    print report
                    report = CollisionReport()
                    in_col = self.env.CheckCollision(self.robot, report)
                    print "in col : ", in_col
                    print report
                    print "q_ik, or q_ik is in collision."
            # if self.useIKFast:
            #     print "Info: using IKFast."
            #     sol0 = self.IKFast('leftArm', array(T0_LH), False)
            #     sol1 = self.IKFast('rightArm', array(T0_RH), False)
            #     if sol0 is not None and sol1 is not None:
            #         self.robot.SetDOFValues(sol0, self.robot.GetManipulators()[0].GetArmIndices())
            #         self.robot.SetDOFValues(sol1, self.robot.GetManipulators()[1].GetArmIndices())
            #     else:
            #         print "Error : IKFast could not find goalik."
            #         return [33, str2num(q_ik)]  # 3: ikfast error, 3: goalik
            #     q_ik = self.robot.GetActiveDOFValues()
            #
            # else:
            #     return [23, q_ik]  # 2: generalik error, 3: at goal ik
        else:
            print "Info : GeneralIK found an ik."
            self.robot.SetActiveDOFValues(str2num.str2num(q_ik))
            self.robot.GetController().Reset(0)
            self.currentikseed = str2num.str2num(q_ik)
        return [0, str2num.str2num(q_ik)]


    def PlanSimpleArmMotion(self, q_goal, active_dofs, tsr_chain_string="", object=None):
        self.robot.SetActiveDOFs(active_dofs)
        q_cur = self.robot.GetActiveDOFValues()
        # Basic CbiRRT variables
        smoothingitrs = 300
        mimicdof = None
        error_code_str = ""
        psample = None if tsr_chain_string == "" else 0.02
        activedofs = active_dofs
        success = False
        with self.env:
            [success, info] = self.PlanTrajectory(q_cur, q_goal, tsr_chain_string, smoothingitrs,
                                                               error_code_str, mimicdof, psample, activedofs)
            print "end planning"
            if success != 1:
                print "planning for simple arm motion failed"
                return [success, info]
        self.DisplayTrajectory()

        return [success, info]


    # Plans a trajectory using a CBiRRT problem and goaljoints (active dofs)
    def PlanTrajectory(self, q_init, q_goal, TSRChainString, smoothingitrs,
                       error_code_str, mimicdof, psample, activedofs ):
        self.robot.GetController().Reset(0)
        # String to number
        if(type(q_init) == type("")):
            q_init = str2num(q_init)

        if(type(q_goal) == type("")):
            q_goal = str2num(q_goal)

        # if self.plan_all_DOF_ik:
        #     # Gets only the active dofs of init and goal
        #     self.robot.SetActiveDOFs(self.alldofs)
        #     self.robot.SetActiveDOFValues(q_goal)
        #     self.robot.GetController().Reset(0)
        #     self.robot.SetActiveDOFs(activedofs)
        #     q_goal = self.robot.GetActiveDOFValues()
        #
        #     self.robot.SetActiveDOFs(self.alldofs)
        #     self.robot.SetActiveDOFValues(q_init)
        #     self.robot.GetController().Reset(0)
        #     self.robot.SetActiveDOFs(activedofs)
        #     q_init = self.robot.GetActiveDOFValues()

        # Convert q_target to numbers
        if(type(q_goal) == type("")):
            goaljoints = deepcopy(str2num.str2num(q_goal))
        else:
            goaljoints = deepcopy(q_goal)

        # Check if the goal config needs to have padding pushed away
        self.robot.SetActiveDOFValues(q_init)
        q_tmp = self.robot.GetDOFValues()

        # Set controler to init config
        t = self.robot.GetTransform()
        self.robot.GetController().SetDesired(q_tmp, t)
        time.sleep(5)
        #self.robot.WaitForController(0)

        # Change to plan with lower number of dofs (onlyArms)
        self.robot.SetActiveDOFs(activedofs)

        # Then add extra dofs for each TSRMimicDOF
        if mimicdof is not None:
            for i in range(mimicdof):
                goaljoints = append(goaljoints, [0], 0)

        cmdStr = 'RunCBiRRT timelimit 60 '

        print "CALL CBIRRT"
        try:
            if psample != None:
                cmdStr += 'psample '+str(psample)+' '

            cmdStr += 'smoothingitrs '+str(smoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainString
            answer = '0'
            answer = self.probs_cbirrt.SendCommand(cmdStr)

            print "RunCBiRRT answer: ", str(answer)
            if answer[0] != '1':
                return [False, error_code_str+" - "+answer[1:]]
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e
            return [False, "CBiRRT Plug-in Exception."]

        return [True, ""]


    # Approaching action to reach the nut when unscrew
    def ApproachingAction(self, object):
        target_object = self.GetObjectByName(object)
        print "Perform Approaching Action"
        self.robot.SetDOFValues([0.0], [self.robot.GetJoint('torso_lift_joint').GetDOFIndex()])
        if object == "nut_LF_1":
            self.robot.SetActiveDOFs(self.r_arm_indices)
        elif object == "wheel_LF":
            self.robot.SetActiveDOFs(self.arm_indices)
        q_init = self.robot.GetActiveDOFValues()

        q = self.robot.GetDOFValues()
        t = self.robot.GetTransform()
        self.robot.GetController().SetDesired(q, t)
        time.sleep(0.1)

        self.SetProblems()
        if object == "nut_LF_1":

            # # move nut deeper on the stud
            # T = target_object.GetTransform()
            # #self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, T, 1))
            # T = T * MakeTransform(eye(3), matrix([0, -0.05, -0.002]))
            # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, T, 1))
            # target_object.SetTransform(asarray(T))
            # # sys.stdin.readline()

            T0_RH = self.GetRightHandTransform()
            T0_LH = self.GetLeftHandTransform()
            end_effector_offset = MakeTransform(rodrigues.rodrigues([0, pi, 0]), matrix([0, 0.0, -0.00]))*MakeTransform(rodrigues.rodrigues([0, 0, pi/2]), matrix([0, 0.0, 0.0])) # turn manip right way to pick a nut from top

            self.drawingHandles.append(misc.DrawAxes(self.env, target_object.GetTransform(), 1))
            self.drawingHandles.append(misc.DrawAxes(self.env, T0_RH, 1))

            print "object and manip transform"
            #sys.stdin.readline()

            T0_RH = target_object.GetTransform()*end_effector_offset

            del self.drawingHandles[:]
            self.drawingHandles.append(misc.DrawAxes(self.env, T0_RH, 1))

        elif object == "wheel_LF":

            # move nut deeper on the stud
            T = target_object.GetTransform()
            #self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, T, 1))
            T = T * MakeTransform(eye(3), matrix([-0.16, -0.0, -0.00]))
            self.drawingHandles.append(misc.DrawAxes(self.env, T, 1))
            target_object.SetTransform(asarray(T))
            # print "new wheel transform is set"
            # sys.stdin.readline()

            [T0_RH, T0_LH] = self.pr2_placement.GetWheelHandPlacements(target_object, 1.18, 0.3, False)

            self.drawingHandles.append(misc.DrawAxes(self.env, T0_RH, 1))
            self.drawingHandles.append(misc.DrawAxes(self.env, T0_LH, 1))

        print "goal transform"
        sys.stdin.readline()

        [error, q_goal] = self.FindTwoArmsIK(T0_RH, T0_LH, False)

        self.robot.SetActiveDOFValues(q_init)
        q = self.robot.GetDOFValues()
        t = self.robot.GetTransform()
        self.robot.GetController().SetDesired(q, t)
        time.sleep(0.1)

        if error != 0:
            print "ERROR IN IK. Press return to exit."
            if self.use_keystrokes : sys.stdin.readline()
            return -1

        [success, info] = self.PlanSimpleArmMotion(q_goal, self.r_arm_indices, "")

        print "end planning"

        if success != 1:
            print "error : approach planning did not succeed"
            print "Press return to exit."
            if self.use_keystrokes: sys.stdin.readline()
            return -1
        #sys.stdin.readline()




    def ReachingAction(self, objects, manip_indicies, Thw_path, Twe_path, Bw_path, T0w_goal, Twn_goal, Bw_goal, verbose = False, use_TSR = True):
        print "Perform Reaching Action 2"

        # print "active dofs"
        # print self.robot.GetActiveDOFValues()
        # sys.stdin.readline()

        del self.drawingHandles[:]

        # initial offset from stud to nut
        Twn_init = T0w_goal

        nut = self.GetObjectByName(objects[0])
        target_object = self.GetObjectByName(objects[1])

        # set stud as the reference (TSR) frame
        T0w_goal = dot(target_object.GetTransform(), MakeTransform(rodrigues.rodrigues([0., 0., -pi/2]), matrix([0.12, 0., 0.0635])))

        # offset from nut to end-effector (includes the initial nut offset from the stud)
        #Tne_goal = MakeTransform(rodrigues([0, pi, 0]), matrix([0, 0.0, -0.0153]))*MakeTransform(rodrigues([0, 0, pi/2]), matrix([0, 0.0, 0.0])) # before Vicon nut adjustment
        Tne_goal = MakeTransform(rodrigues.rodrigues([0, pi, 0]), matrix([0, 0.0, -0.00]))*MakeTransform(rodrigues.rodrigues([0, 0, pi/2]), matrix([0, 0.0, 0.0]))
        # offset from TSR (stud) frame to end-effector
        Twe_goal = dot(Twn_goal, Tne_goal)

        # print "Current nut transform", nut.GetTransform()
        # print "Current nut in drawn"
        # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, nut.GetTransform(), 1))
        # sys.stdin.readline()
        #
        # print "Vicon nut transform", T0w_goal*Twn_init
        # print "Vicon nut in drawn"
        # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, T0w_goal*Twn_init, 1))
        # sys.stdin.readline()
        #
        # print "Set nut transform as in Vicon"
        # nut.SetTransform(asarray(T0w_goal*Twn_init))
        # sys.stdin.readline()



        if verbose:
            print "T0w_goal \n", T0w_goal
            print "Twn_goal \n", Twn_goal
            print "Tne_goal \n", Tne_goal
            print "Twe_goal \n", Twe_goal
            print "Bw_goal \n", Bw_goal
            print "Goal (nut) in the world frame \n", dot(T0w_goal, Twn_goal)
            print "Goal (hand) in the world frame \n", dot(T0w_goal, Twe_goal)
            print "Right hand frame \n",self.GetRightHandTransform()

            print "T0w_goal drawn"
            self.drawingHandles.append(misc.DrawAxes(self.env, T0w_goal, 1))
            sys.stdin.readline()

            # print "Twn_goal in drawn"
            # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, Twn_goal, 1))
            # sys.stdin.readline()

            # print "T0n_goal in drawn"
            # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(T0w_goal, Twn_goal), 1))
            # sys.stdin.readline()

            # print "nut current in drawn"
            # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, nut.GetTransform(), 1))
            # sys.stdin.readline()

            print "Tne_goal in drawn"
            self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, Tne_goal, 1))
            sys.stdin.readline()

            print "Twe_goal in TSR frame drawn"
            self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, Twe_goal, 1))
            sys.stdin.readline()

            self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(T0w_goal, Twe_goal), 1))
            print "Twe_goal goal in world frame is drawn"
            sys.stdin.readline()

            self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, self.pr2_planning.GetRightHandTransform(), 1))
            print "EE current frame is drawn"
            sys.stdin.readline()

        del self.drawingHandles[:]

        # # TODO put this code into TSR extraction

        # T0w_path = dot(T0w_goal, Thw_path)
        # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, T0w_path, 1))
        # print "T0w_path drawn"
        # sys.stdin.readline()

        #Thw_path[0:3, 0] *= -1;
        T0w_path = dot(T0w_goal, Thw_path)

        # Thw_path[0][:,0:3] = Thw_path[0][:,0:3] * -1;
        # T0w_path = dot(T0w_goal, Thw_path) # TSR transform in the world frame
        # print "Det: "
        # print numpy.linalg.det(Thw_path[0:3][:,0:3])
        # sys.stdin.readline()
        #Twe_path = MakeTransform(dot(Inverse(T0w_path), self.pr2_planning.GetRightHandTransform())[0:3][:,0:3], matrix([0., 0., 0.0]))* MakeTransform(eye(3), matrix([0.00, 0.0003, .0173])) # before Vicon nut adjustment
        Twe_path = MakeTransform(dot(Inverse(T0w_path), self.GetRightHandTransform())[0:3][:,0:3], matrix([0., 0., 0.0]))
        #print "Twe_path \n", Twe_path
        #Twe_path_new = dot(Inverse(T0w_path), self.pr2_planning.GetRightHandTransform())
        #print "Twe_path_new \n", Twe_path_new
        #sys.stdin.readline()

        if verbose:

            print "T0w_path \n", T0w_path
            print "Twe_path \n", Twe_path
            print "Bw_path \n", Bw_path
            print "Right hand transform in world frame  \n", self.pr2_planning.GetRightHandTransform()
            print "Right hand transform in TSR frame  \n", dot(self.pr2_planning.GetRightHandTransform(), Inverse(Twe_path))
            print "Goal transform in TSR frame  \n", dot(dot(T0w_goal, Twe_goal), Inverse(Twe_path))
            #print "Goal new transform in TSR frame  \n", dot(dot(T0w_goal, Twe_goal), Inverse(Twe_path_new))

            self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, T0w_path, 1))
            print "T0w_path drawn"
            sys.stdin.readline()

            # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, Twe_path, 1))
            # print "Twe_path in TSR frame drawn"
            # sys.stdin.readline()
            #
            self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(T0w_path, Twe_path), 1))
            print "T0e in world frame drawn"
            sys.stdin.readline()

            self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, self.pr2_planning.GetRightHandTransform(), 1))
            print "Right hand transform in world frame is drawn"
            sys.stdin.readline()

            #self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(Inverse(T0w_path), self.pr2_planning.GetRightHandTransform()), 1))
            self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(self.pr2_planning.GetRightHandTransform(), Inverse(Twe_path)), 1))
            print "Right hand transform in TSR frame is drawn"
            sys.stdin.readline()

            # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(self.pr2_planning.GetRightHandTransform(), Inverse(Twe_path_new)), 1))
            # print "Right hand transform new in TSR frame is drawn"
            # sys.stdin.readline()

            self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(dot(T0w_goal, Twe_goal), Inverse(Twe_path)), 1))
            print "Goal transform in TSR frame is drawn"
            sys.stdin.readline()

            # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(dot(T0w_goal, Twe_goal), Inverse(Twe_path_new)), 1))
            # print "Goal transform in TSR new frame is drawn"
            # sys.stdin.readline()

            self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(T0w_goal, Twe_goal), 1))
            print "Twe_goal goal in world frame is drawn"
            sys.stdin.readline()

        if False:
            newrobot = RaveCreateRobot(self.pr2_planning.env, self.robot.GetXMLId())
            newrobot.Clone(self.robot,0)

            self.robot.SetActiveDOFValues([-0.362927, -0.325898, -1.28184, -0.818685, -1.4624, -1.94644, -2.57304])

            for link in newrobot.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(0.7)

            self.pr2_planning.env.Add(newrobot,True)
            newrobot.SetTransform(self.robot.GetTransform())
            newrobot.SetDOFValues(self.robot.GetDOFValues())
            #self.robot.SetActiveDOFs(arm_indicies)

            q = newrobot.GetDOFValues()
            t = newrobot.GetTransform()
            newrobot.GetController().SetDesired(q, t)
            time.sleep(0.1)
            print "manipulator set to the projected configuration"
            sys.stdin.readline()

        # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, self.pr2_planning.GetRightHandTransform(), 1))
        # print "Projected manipulator config is world frame"
        # sys.stdin.readline()
        #
        # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(self.pr2_planning.GetRightHandTransform(), Inverse(Twe_path)), 1))
        # print "Projected manipulator config in TSR frame is drawn"
        # sys.stdin.readline()
        #
        # print "Projected manipulator in world frame  \n", self.pr2_planning.GetRightHandTransform()
        # print "Projected manipulator in TSR frame  \n", dot(self.pr2_planning.GetRightHandTransform(), Inverse(Twe_path))


        # del self.pr2_planning.drawingHandles[:]
        # # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, T0w_path, 1))
        # # print "T0w_path drawn"
        # print "ee pose in the TSR path frame"
        # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(T0w_path, Twe_path), 1))
        # print dot(T0w_path, Twe_path)
        # sys.stdin.readline()
        #
        # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, self.pr2_planning.GetRightHandTransform(), 1))
        # print "Right hand transform is drawn"
        # sys.stdin.readline()

        print "Place robot in the right position"
        self.goal_tsrs = []
        del self.goal_tsrs[:]

        arm_indicies = array([])
        if manip_indicies.count(5):
            arm_indicies = self.r_arm_indices
        if manip_indicies.count(7):
            arm_indicies = concatenate( (arm_indicies, self.l_arm_indices), axis=0 )

        print arm_indicies

        end_effector_offset = Twe_goal
        #end_effector_offset = matrix([[1,0,0,0],[0,0,-1,-0.02],[0,1,0,0], [0,0,0,1]])

        self.goal_tsrs.append(self.GetRightHandTransform())

        if True: # code from old reach action

            self.robot.SetDOFValues([0.0], [self.robot.GetJoint('torso_lift_joint').GetDOFIndex()])

            self.robot.SetActiveDOFs(arm_indicies)
            q_init = self.robot.GetActiveDOFValues()

            self.SetProblems()
            with self.env:


                # default right and left enf-effectors' transforms
                T0_RH = self.GetRightHandTransform()
                T0_LH = self.GetLeftHandTransform()

                # TODO check condition on which end-effector do we use
                if manip_indicies.count(5):
                    T0_RH = target_object.GetTransform()*end_effector_offset

                # if manip_indicies.count(7):
                #     print "TODO Define left end effector transforms"
                #     T0_LH = matrix([[1,0,0,0.],[0,1,0,0],[0,0,1,0.], [0,0,0,1]])*target_object.GetTransform()*end_effector_offset

            # [error, q_goal] = self.pr2_planning.FindTwoArmsIK(T0_RH, T0_LH, False)\

            # [T0_RH, T0_LH] = self.pr2_placement.GetWheelHandPlacements(target_object, 0, 0.30, False)

            print "Start RRT (q_init)"
            self.robot.SetActiveDOFValues(q_init)
            # T0_RH1 = target_object.GetTransform()*end_effector_offset

            # print end_effector_offset
            #print Inverse(end_effector_offset)
            # print target_object
            del self.drawingHandles[:]

            #self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, self.pr2_planning.GetRightHandTransform(), 1))
            # [error, q_goal] = self.pr2_planning.FindTwoArmsIK(T0_RH, T0_LH, False)
            # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, T0_LH, 1))

            # TSRstring0 = SerializeTSR(manip_indicies[0], 'NULL', T0_RH, mat(eye(4)), boundary_matrix)

            #TSRstring0 = SerializeTSR(manip_indicies[0], 'NULL', target_object.GetTransform(), end_effector_offset, Bw_goal)

            #TSRChainString = '%s' % SerializeTSRChain(0, 1, 0, 2, TSRstring1 + ' ' + TSRstring0, 'NULL', [])
            #TSRChainString = '%s' % SerializeTSRChain(0, 1, 0, 1, TSRstring0, 'NULL', [])


            #Bw_path = mat([-0.015, 0.015, -0.015, 0.015, -1000, 1000., -1000, 1000., -1000, 1000., -1000, 1000.])
            # TODO add the boundaries noise while extracting TSR
            Bw_goal = mat([-0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0, 0., -0, 0., -0, 0.])
            Bw_path = mat([-0.02, 0.02, -0.02, 0.02, -1000, 1000., -1000, 1000., -1000, 1000., -1000, 1000.]) #-0.04, 0.04, -0.08, 0.08,
            #Bw_path = mat([-0.2, 0.2, -0.2, 0.2, -1000, 1000., -1000, 1000., -1000, 1000., -1000, 1000.]) #-0.04, 0.04, -0.08, 0.08,
            TSRstring_path = SerializeTSR(manip_indicies[0], 'NULL', T0w_path, Twe_path, Bw_path)

            # TODO added rodrigues rotation to compensate for invalid hand rotation in Vicon
            # TSRstring_goal = SerializeTSR(manip_indicies[0], 'NULL', target_object.GetTransform(), end_effector_offset*MakeTransform(rodrigues([0, pi, 0]), matrix([0, 0, -0.05])), mat([-0, 0, -0, 0, -0, 0, -0, 0, -0, 0, -0, 0])) #Bw_goal)
            TSRstring_goal = SerializeTSR(manip_indicies[0], 'NULL',  T0w_goal, Twe_goal, Bw_goal) #Bw_goal)


            # pack the TSRs into a string
            TSRChainString = '%s %s' % (SerializeTSRChain(0, 0, 1, 1, TSRstring_path, 'NULL', []), SerializeTSRChain(0, 1, 0, 1, TSRstring_goal, 'NULL', []))
            #TSRChainString = '%s' % ( SerializeTSRChain(0, 1, 0, 1, TSRstring_goal, 'NULL', []))

            if False:
                q_init = self.robot.GetActiveDOFValues()

                self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(T0w_goal, Twe_goal), 1))
                self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, self.pr2_planning.GetLeftHandTransform(), 1))

                [error, q_goal] = self.pr2_planning.FindTwoArmsIK(dot(T0w_goal, Twe_goal), self.pr2_planning.GetLeftHandTransform(), False)

                print "IK solution for a goal found"
                print q_goal

                if error != 0:

                    # WHY ????
                    q = self.robot.GetDOFValues()
                    t = self.robot.GetTransform()
                    self.robot.GetController().SetDesired(q, t)
                    time.sleep(0.1)
                    #self.robot.WaitForController(0)PlanSimpleArmMotion

                    print "Press return to exit."
                    if self.use_keystrokes : sys.stdin.readline()
                    return -1

                self.robot.SetActiveDOFValues(q_init)


            #TSRChainString = '%s' % ( SerializeTSRChain(0, 0, 1, 1, TSRstring_path, 'NULL', []))

            # print "press enter to plan"
            # sys.stdin.readline()
            #
            self.robot.Grab(nut)
            #
            # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, self.pr2_planning.GetRightHandTransform(), 1))
            # print "Right hand transform in world frame is drawn"
            # sys.stdin.readline()
            #
            # self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(T0w_goal, Twe_goal), 1))
            # print "Twe_goal goal in world frame is drawn"
            # sys.stdin.readline()

            #self.pr2_planning.drawingHandles.append(misc.DrawAxes(self.pr2_planning.env, dot(T0w_goal, Twe_goal), 1))
            # print "Twe_goal goal in world frame is drawn"
            # sys.stdin.readline()

            #del self.pr2_planning.drawingHandles[:]
            if False:
                from random import *
                for i in range(100):
                    Twsample = MakeTransform(eye(3), matrix([uniform(Bw_path[0,0], Bw_path[0,1]), uniform(Bw_path[0,2], Bw_path[0,3]), uniform(-0.1, 0.2)]))
                    print "Twsample \n", Twsample
                    T0sample = dot(dot(T0w_path, Twsample), Twe_path)
                    #T0sample = dot(T0w_path, Twsample)
                    print "T0sample \n", T0sample
                    self.drawingHandles.append(misc.DrawAxes(self.env, T0sample, 1))
                    sys.stdin.readline()


            if use_TSR:
                self.PlanSimpleArmMotion([], arm_indicies, TSRChainString, target_object)

                # print self.robot.GetActiveDOFValues()
                # print nut.GetTransform()
                # sys.stdin.readline()

            else:
                print "not using constraints"

                q_init = self.robot.GetActiveDOFValues()

                self.drawingHandles.append(misc.DrawAxes(self.env, dot(T0w_goal, Twe_goal), 1))
                self.drawingHandles.append(misc.DrawAxes(self.env, self.GetLeftHandTransform(), 1))

                [error, q_goal] = self.FindTwoArmsIK(dot(T0w_goal, Twe_goal), self.GetLeftHandTransform(), False)

                #sys.stdin.readline()

                if error != 0:

                    # WHY ????
                    q = self.robot.GetDOFValues()
                    t = self.robot.GetTransform()
                    self.robot.GetController().SetDesired(q, t)
                    time.sleep(0.1)
                    #self.robot.WaitForController(0)PlanSimpleArmMotion

                    print "IK solution was not found"

                self.robot.SetActiveDOFValues(q_init)
                # WHY ????
                q = self.robot.GetDOFValues()
                t = self.robot.GetTransform()
                self.robot.GetController().SetDesired(q, t)
                self.robot.WaitForController(0)
                #TODO remove when not drawing
                time.sleep(0.2)
                print "planning"
                [success, info] = self.PlanSimpleArmMotion(q_goal, self.r_arm_indices, "", nut)
            print "End planning"
        return 0


    def plan_request(self, request):
        self.filename = request[0]
        self.current_objects = request[1]
        file_to_read = open(self.filename, "rb")
        loaded_list = pickle.load(file_to_read)
        if self.filename == "Unscrew":
            self.ApproachingAction(self.current_objects[0])
        for i in range(len(loaded_list)):
            self.ReachingAction(self.current_objects, loaded_list[i][1], loaded_list[i][3], loaded_list[i][4], loaded_list[i][5],
                                loaded_list[i][6], loaded_list[i][7], loaded_list[i][8])


if __name__ == "__main__":
    planning = TSR_exec("models/heres_how_env.xml")
    TSR_filename = 'Unscrew'
    planning_request = [TSR_filename, ["nut_LF_1", "hub_LF"]]
    planning.plan_request(planning_request)
    raw_input("Press enter to exit...")

