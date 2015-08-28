'''
!!!!!!!!!!!!!!!!!!!!!!

Author: Artem Gritsenko
Worcester Polytechnic Institute, ArcLab
July 2015
'''

import csv
import sys

from openravepy import *
import numpy as np
import scipy
from wpi_planning_utilities.transformation_helper import *

class TrajectoryProcessor():

    def __init__(self, OpenRaveEnv):
        self.drawingHandles = []
        self.env = OpenRaveEnv


    def split_demonstration(self, demonstration_path, demonstration_filename, seg_filename):
        with open(seg_filename, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            action_names = reader.next()
            start_points = reader.next()
            end_points = reader.next()
        start_points = [int(i) for i in start_points]
        end_points = [int(i) for i in end_points]
        data = []
        with open(demonstration_path+demonstration_filename, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for row in reader:
                data.append([float(i) for i in row])
        for i in range(len(action_names)):
            # write action into csv file
            action_csv = demonstration_path + action_names[i] + '_' + demonstration_filename
            action_points = data[start_points[i]:end_points[i]]
            open(action_csv, 'w').close()
            with open(action_csv, 'a') as csvfile:
                writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                for e in action_points:
                    writer.writerow(e)
        return action_names


    def read_splits(self, filename):
        splits = []
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for row in reader:
                splits.append(int(row[0]))
        return splits


    def get_holding_info(self, filename):
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            hold_labels = reader.next()
            start_points = reader.next()
            end_points = reader.next()
        holding_info = []
        for i in range(len(hold_labels)):
            holding_info = holding_info + [int(hold_labels[i]) for x in range(int(start_points[i]), int(end_points[i]))]
        return holding_info


    def get_positions_and_transforms_from_csv_bag(self, filename, verbose = False):
        points = zeros((0, 3))
        transforms = []
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for row in reader:
                points = append(points, [np.asarray([float(row[0]), float(row[1]), float(row[2])])], axis = 0)
                new_transform = BuildMatrix([float(row[i]) for i in range(3)], [float(row[i]) for i in range(3,len(row))])
                transforms.append(new_transform)
        return points, transforms


    def remove_duplicates(self, points, transforms, verbose = False):
        new_points = []
        new_transforms = []
        if len(points) > 0:
            new_points.append(points[0])
            new_transforms.append(transforms[0])
            for i in range(1, len(points)):
                if not (points[i] == points[i-1]).all():
                    new_points.append(points[i])
                    new_transforms.append(transforms[i])
                elif verbose: print "point %r removed" % i
        if verbose: print "Number of points before removing duplicates is: ", len(points)
        if verbose: print "Number of points after removing duplicates is: ", len(new_points)
        return new_points, new_transforms


    def create_straight_line(self, points):
        incr_x = (points[-1][0] - points[0][0])/len(points)
        incr_y = (points[-1][1] - points[0][1])/len(points)
        incr_z = (points[-1][2] - points[0][2])/len(points)
        for i in range(1, len(points)-1):
            points[i] = points[0] + i * np.array([incr_x, incr_y, incr_z])
        return points


    def resampling_3D_curve(self, ee_points, num_samples = 100, verbose = False):
        # doing the interpolation
        intp = scipy.interpolate.SmoothBivariateSpline(ee_points[:,0], ee_points[:,1], ee_points[:,2])
        # calculate length of intervals of the curve
        lengths = [scipy.spatial.distance.euclidean(ee_points[i], ee_points[i+1]) for i in range(0, len(ee_points)-1)]
        lengths.insert(0, 0.)
        #calculate cumulative and total length
        cum_lengths = []
        current_length = 0
        for l in lengths:
            current_length += l
            cum_lengths.append(current_length)
        total_length = current_length
        if verbose: print "Total length of the curve is %r." % (cum_lengths)
        # calculate the new trajectory by resampling based on a trajectory length
        resampled_curve = []
        resampled_curve.append(ee_points[0])
        cum_dist = 0
        matching = {} # dictionary to keep the matching between new (resampled) and old points
        matching[0] = 0
        for i in range(1, num_samples):
            cum_dist += total_length/num_samples
            idx = -1
            for j in range(1, len(cum_lengths)):
                if cum_lengths[j] > cum_dist:
                    idx = j
                    break
            if idx == -1:
                matching[i] = num_samples-1
                break
            dx = ee_points[idx, 0] - ee_points[idx-1, 0]
            dy = ee_points[idx, 1] - ee_points[idx-1, 1]
            ratio = (cum_dist - cum_lengths[idx-1])/(cum_lengths[idx] - cum_lengths[idx-1])
            matching[i] = idx if ratio > 0.5 else idx-1 # match a new point to the old one
            new_x = ee_points[idx-1, 0] + dx * ratio
            new_y = ee_points[idx-1, 1] + dy * ratio
            new_z = intp.ev(new_x, new_y)
            new_point = np.array([new_x, new_y, new_z])
            resampled_curve.append(new_point)
        return resampled_curve, matching


    def draw_points(self, points, color = (0, 0, 1), by_point = False):
        if by_point:
            for i in range(len(points)):
                self.drawingHandles.append(self.env.plot3( points = np.asarray(points[i]), pointsize=3.0, colors = np.asarray([color]) ))
                print "point %r is drawn" %i
                sys.stdin.readline()
        else:
            colors = []
            [colors.append(color) for x in xrange(0, len(points))]
            self.drawingHandles.append(self.env.plot3( points = np.asarray(points), pointsize=3.0, colors = np.asarray(colors) ))


    def clear_drawing(self):
        del self.drawingHandles[:]