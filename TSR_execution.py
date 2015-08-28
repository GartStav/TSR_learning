#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code based on the simplemanipulation.py example
import time
import csv
import sys
import os
import openravepy
import numpy as np
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

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

drawingHandles = []
TSR_filename = 'TSR_segmentation/TSRs/ScrewUnscrewDemonstrationsArtem2/Unscrew'
traj_id = 0

def waitrobot(robot):

    q = robot.GetDOFValues()
    t = robot.GetTransform()
    robot.GetController().SetDesired(q, t)
    robot.WaitForController(0)

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

def DisplayTrajectory(env, robot, probs_cbirrt):

    global traj_id

    print "displaying trajectory ", traj_id

    answers = []
    try:
        traj = RaveCreateTrajectory(env, '')
        f = open('cmovetraj.txt', 'r')
        shutil.copy2('cmovetraj.txt', 'trajs/cmovetraj' + str(traj_id).zfill(4) + '.txt')
        traj.deserialize(f.read())
        f.close()

        traj_id += 1

        # if do_screw_motion:
        #     [traj, q_end] = self.AddScrewMotion(traj)

        # self.robot.GetController().SetPath(traj)

        answers.append(probs_cbirrt.SendCommand('traj cmovetraj.txt'))
        print "traj call answer: ", str(answers[-1])

        robot.WaitForController(0)

        # if do_screw_motion:
        #     self.robot.SetDOFValues(q_end)

    except openrave_exception, e:
        print e
        return [False, "OS exception in PlayTrajectory."]

    robot.GetController().Reset(0)

    # print "Press return to exit."
    # sys.stdin.readline()

    q = robot.GetDOFValues()
    t = robot.GetTransform()
    robot.GetController().SetDesired(q, t)
    robot.WaitForController(0)

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

    probs_cbirrt = RaveCreateModule(env, 'CBiRRT')

    try:
        env.AddModule(probs_cbirrt, robot.GetName())
    except openrave_exception, e:
        print e

    print "Getting Loaded Problems"

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


if __name__ == "__main__":

    print "Setting up the environment..."
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('robots/pr2-beta-static.zae')
    robot = env.GetRobots()[0]
    time.sleep(0.1)

    file_to_read = open(TSR_filename, "rb")
    loaded_list = pickle.load(file_to_read)

    traj_all = RaveCreateTrajectory( env, '')
    traj_all.Init(robot.GetActiveConfigurationSpecification())
    trajs_path = 'TSR_segmentation/trajs/Unscrew_Screw_LF_1_limits'
    traj_list = sorted(os.listdir(trajs_path))
    print traj_list
    f = open(traj_list[0], 'r')
    traj_all.deserialize(f.read())

    for j in range(traj_all.GetNumWaypoints()):
        data = traj_all.GetWaypoint(j)
        # extract the robot joint values only
        robot.SetDOFValues(data[:39])
        break

    print loaded_list
    sys.stdin.readline()

    for i in range(len(loaded_list)):
        PlanMotion(env, robot, loaded_list[i][1], loaded_list[i][2], loaded_list[i][3], loaded_list[i][4],
                                loaded_list[i][5], loaded_list[i][6], loaded_list[i][7])



    raw_input("Press enter to exit...")

