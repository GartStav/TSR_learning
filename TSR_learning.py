#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code based on the simplemanipulation.py example
import time
import csv
import sys
import os
import openravepy
import numpy as np

import scipy
from scipy import signal
import pickle

from matplotlib.mlab import PCA as mlabPCA

from wpi_planning_utilities.transformation_helper import *

# drawing
from matplotlib import pyplot as plt
from wpi_planning_utilities.TransformMatrix import *
from wpi_planning_utilities.rodrigues import *
from scipy.spatial import ConvexHull

import openravepy

from traj_processor import TrajectoryProcessor
from traj_analyzer import TrajectoryAnalyzer

import mlpy

from itertools import tee, izip

drawingHandles = []
groundtruth_filename = 'data/PCA_groundtruth_idx.csv'
#groundtruth_filename = 'TSR_segmentation/data/PCA_groundtruth_idx_2.csv'
PCA_results_filename = 'data/PCA_z_axis.csv'
#PCA_results_filename = 'TSR_segmentation/data/PCA_z_axis_2.csv'
trajs_path = 'trajs/Unscrew_Screw_LF_1_limits'
#trajs_path = 'TSR_segmentation/trajs/Unhang_and_hang'
recovered_paths = 'trajs/Unscrew_recovered_4'
#bag_name  = "ScrewUnscrewDemonstrationsArtem1"
# bags_file = "bags_csv/" + bag_name + "/nut_" + bag_name + ".csv"
bags_file = "bags_csv/ArtemUnscrewScrewNut-1/nut_ArtemUnscrewScrewNut-1.csv"

TSR_filename = 'Rec_Task_bag_2'
#TSR_filename = 'TSR_segmentation/Rec_Task_2'

import numpy as np

class TSRExtractor():

    def __init__(self, extract_from_bags = True):
        self.setup_openrave()
        self.use_bags = extract_from_bags

        self.traj_proc = TrajectoryProcessor(self.env)
        self.analyzer = TrajectoryAnalyzer(self.env)
        self.num_samples = 100 # number of points to resample the trajectories

        # TODO IMPORT FILENAMES AS PARAMETERS
        self.bag_name  = "UnhangHangWithoutHandArtem1" # filename of a bag with demonstration
        self.action_seg_name = "action_segmentation.txt" # filename with segmentation points between actions
        self.hold_seg_name_postfix = "_segmentation_holding.txt" # part of a filename (postfix) for each action holding/not holding segmentation points


    def setup_openrave(self):
        print "Setting up the environment..."
        self.env = openravepy.Environment()
        self.env.SetViewer('qtcoin')
        self.env.Reset()
        self.env.Load('robots/pr2-beta-static.zae')
        self.robot = self.env.GetRobots()[0]
        self.robot.SetActiveManipulator("rightarm")
        T = self.robot.GetTransform()
        T[0, 3] -= 2
        self.robot.SetTransform(T)
        time.sleep(0.1)


    def extract_TSR(self, action):

        nut_points, nut_trans = self.traj_proc.get_positions_and_transforms_from_csv_bag("bags_csv/" + self.bag_name + "/" + action + "_nut_" + self.bag_name + ".csv")
        hand_points, hand_trans = self.traj_proc.get_positions_and_transforms_from_csv_bag("bags_csv/" + self.bag_name + "/" + action + "_hand_" + self.bag_name + ".csv")

        extractor.traj_proc.draw_points(nut_points, by_point=False)
        extractor.traj_proc.draw_points(hand_points, by_point=False)
        sys.stdin.readline()
        extractor.traj_proc.clear_drawing()

        if False:
            holding = self.traj_proc.get_holding_info("bags_csv/" + self.bag_name + "/" + action + self.hold_seg_name_postfix)


            # resample trajectories
            nut_points_resampled, nut_matching = self.traj_proc.resampling_3D_curve(nut_points, self.num_samples)
            hand_points_resampled, hand_matching = self.traj_proc.resampling_3D_curve(hand_points, self.num_samples)

            # print nut_matching
            # print hand_matching
            # print len(holding)
            # sys.stdin.readline()

            # TODO use matching to find these vars
            nut_trans_resampled = [nut_trans[nut_matching[i]] for i in range(self.num_samples)]
            hand_trans_resampled = [hand_trans[hand_matching[i]] for i in range(self.num_samples)]
            holding_resampled_1 = [holding[nut_matching[i]] for i in range(self.num_samples)]
            holding_resamled_2 = [holding[hand_matching[i]] for i in range(self.num_samples)]

            extractor.traj_proc.draw_points(nut_points_resampled)
            extractor.traj_proc.draw_points(hand_points_resampled)
            #extractor.traj_proc.clear_drawing()

            # run PCA to find segmentation points
            #self.analyzer.find_segmentation_points(nut_points_resampled, to_plot = True)
            self.analyzer.find_segmentation_points(hand_points_resampled, to_plot = True)

            print "segmentation graphs plotted"
            sys.stdin.readline()

            # print holding_resampled_1
            # print holding_resamled_2
            # sys.stdin.readline()

            # print len(nut_trans_resampled)
            # sys.stdin.readline()
            #
            # for i in nut_trans_resampled:
            #     print i
            #     sys.stdin.readline()


            # extractor.traj_proc.draw_points(nut_points_resampled, by_point=True)
            # extractor.traj_proc.draw_points(hand_points_resampled, by_point=True)
            # extractor.traj_proc.clear_drawing()
            #
            # sys.stdin.readline()

            # 2. run PCA to find segmentation points
            # 3. TODO pretend found the correct segmientation, store it the file and read from it
            # 4. run PCA again to extract TSRs
            # 5. write them into file

            # print "nut points"
            # print nut_points
            # print "hand points"
            # print hand_points
            #
            # print len(nut_points), len(hand_points)
            # sys.stdin.readline()
            # self.traj_proc.draw_points(nut_points)



if __name__ == "__main__":

    extractor = TSRExtractor()

    object_ = "wheel"

    # nut_points, nut_trans = extractor.traj_proc.get_positions_and_transforms_from_csv_bag("bags_csv/" + extractor.bag_name + "/" + object_ + "_" + extractor.bag_name + ".csv")
    # extractor.traj_proc.draw_points(nut_points, by_point=True)
    # sys.stdin.readline()

    print "Splitting demonstration into action files"
    #action_list = extractor.traj_proc.split_demonstration("bags_csv/" + extractor.bag_name + "/", "hand_" + extractor.bag_name + ".csv", "bags_csv/" + extractor.bag_name + "/" + extractor.action_seg_name)
    action_list = extractor.traj_proc.split_demonstration("bags_csv/" + extractor.bag_name + "/", "wheel_" + extractor.bag_name + ".csv", "bags_csv/" + extractor.bag_name + "/action_segmentation.txt")
    print "Splitting done"

    sys.stdin.readline()

    for action in action_list:
        print "Analyzing action %r" % action
        extractor.extract_TSR(action)




    #print "nut points"
    #print nut_points

    #print "head points"
    #print hand_points

    #nut_points, nut_trans = traj_proc.remove_duplicates(nut_points, nut_trans)
    extractor.traj_proc.draw_points(nut_points, by_point=True)

    print "nut trajectory drawn"
    sys.stdin.readline()

    hand_points, hand_trans = traj_proc.remove_duplicates(hand_points, hand_trans)
    extractor.traj_proc.draw_points(hand_points, color = (1, 0, 0), by_point=True)

    print "hand trajectory drawn"
    sys.stdin.readline()

    time.sleep(0.1)

    to_plot = True

    for window_size in [10]:

        z_axis_windowed = []
        open(PCA_results_filename, 'w').close()

        print "Compute segmentation for window size of " + str(window_size)

        print "number of ee points", len(ee_points)
        iteration = 0
        p_const = [] # variable to keep TSR for comparison
        for each in sliding_window(ee_points, window_size):
            print iteration
            iteration += 1

            window_points = asarray(each)

            z_axis, path_constr, goal_constr = run_PCA(window_points, ee_trans, holding, stud_offset = [0], plot = False, draw_axes = True, verbose = False)

            p_const.append(path_constr)

            z_axis_windowed.append(z_axis)

            with open(PCA_results_filename, 'a') as csvfile:
                writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(z_axis)

            # print "new step of sliding window" #change point in 21 for resampled straight line, 24 for resampled noisy traj
            # sys.stdin.readline()

        #print p_const[0]
        #print p_const[1]
        #raw_input("Press ENTER to go further..")

        # dot products of a sliding window of consequtive z-vectors
        dot_products = np.zeros((len(z_axis_windowed), 7))
        for i in range(3, len(z_axis_windowed)-3):
            # for j in range(6):
            #     dot_products[i, j] = z_axis_windowed(i,:).dot(z_axis_windowed(i-j,:))
            #     dot(data(i,:), data(i-3,:))
            #     print i, j
            dot_products[i][0] = z_axis_windowed[i][:].dot(z_axis_windowed[i-3][:])
            dot_products[i][1] = z_axis_windowed[i][:].dot(z_axis_windowed[i-2][:])
            dot_products[i][2] = z_axis_windowed[i][:].dot(z_axis_windowed[i-1][:])
            dot_products[i][3] = z_axis_windowed[i][:].dot(z_axis_windowed[i+1][:])
            dot_products[i][4] = z_axis_windowed[i][:].dot(z_axis_windowed[i+2][:])
            dot_products[i][5] = z_axis_windowed[i][:].dot(z_axis_windowed[i+3][:])
            dot_products[i][6] = (abs(dot_products[i, 1]) + abs(dot_products[i, 2]) + abs(dot_products[i, 2])
                + abs(dot_products[i, 3]) + abs(dot_products[i, 4]) + abs(dot_products[i, 5]) )/6
            # dot_products[i][6] = min(abs(dot_products[i, 1]), abs(dot_products[i, 2]),abs(dot_products[i, 2]),
            #     abs(dot_products[i, 3]) ,abs(dot_products[i, 4]) ,abs(dot_products[i, 5]) )
            # dot_products[i][6] = abs(dot_products[i, 3])

        for i in range(len(dot_products)):
            print i, dot_products[i]


        # remove 3 first and last zero points (to scale the graph)
        dot_products = dot_products[3:(len(dot_products)-3)]

        if to_plot:
            plt.figure(window_size)
            plt.plot(range(len(dot_products)), dot_products[:, 6], color='blue', alpha=0.5)
            if not extract_from_bags:
                for split in groundtruth_splits:
                    plt.plot([split, split], [0, 1], '-', color='red', alpha=0.5)
            plt.xlabel('trajectory waypoints')
            plt.ylabel('z axis dot products')
            plt.title('Segmentation results for window of size ' + str(window_size))
            plt.show(block=False)

        raw_input("Press ENTER to calculate changepoints..")

        #neg_dot_prod_z = [-i for i in dot_products[:, 6]]

        # if to_plot:
        #     plt.plot(range(len(dot_products)), neg_dot_prod_z, color='red', alpha=0.5)
        #     plt.show()
        #peakind = scipy.signal.find_peaks_cwt(neg_dot_prod_z, np.arange(1, 10))
        #print peakind, dot_products[peakind]
        #change_points = [20, 53, 73, 82, len(dot_products)] #unscrew 1
        #change_points = [20, 37, 67, 85, 96, len(dot_products)] #unscrew 2
        #change_points = [20, 34, 66, 77, 83, len(dot_products)] #unscrew 3
        #change_points = [14, 21, 36, 76, 87, len(dot_products)] #unscrew 4
        change_points = [21, 38, 42, 48, len(dot_products)] #bag 3 screw

        #change_points = [112, 146, 159, 189, 207, 220, 286, 329, 372, 427, 536, len(dot_products)] #unhang
        print change_points

        raw_input("Press ENTER to write the TSR's into file..")

        last_ind = 0
        open(TSR_filename, 'w').close()

        list_of_actions = []

        print ee_points[0]

        for i in change_points:
            current_points = asarray(ee_points[last_ind:i])
            current_transf = ee_trans[last_ind:i]
            current_holding = holding[last_ind:i]
            current_stud = stud_trans[last_ind:i]
            last_ind = i
            print current_points.shape

            hold = True if current_holding[1] == 1 else False
            #print hold
            #print current_stud
            sys.stdin.readline()
            z_axis, path_constr, goal_constr = run_PCA(current_points, current_transf, current_holding, current_stud, plot = False, draw_axes = False, verbose = False)

            print goal_constr[1]
            raw_input("Press enter for the next segment...")

            list_of_actions.append(["R", [5], hold, path_constr[0], path_constr[1], path_constr[2],
                                   goal_constr[0], goal_constr[1], goal_constr[2]])

        pickle.dump(list_of_actions, open(TSR_filename, "wb"))

    raw_input("Press enter to exit...")

