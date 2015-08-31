#!/usr/bin/env python
'''
TSR_extraction.py
Extract TSRs (Task Space Region constraint representation) from Rosbag files or set of OpenRave trajectories.
All file paths and parameters are init as global variables at the begining of a script.

Author: Artem Gritsenko
Worcester Polytechnic Institute, ArcLab
July 2015
'''

import time
import csv
import sys
import os
from openravepy import *

import numpy as np
import scipy
import pickle

# drawing
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch

from wpi_planning_utilities.TransformMatrix import *
from wpi_planning_utilities.rodrigues import *
from wpi_planning_utilities.transformation_helper import *

from scipy.spatial import ConvexHull
from scipy import signal

import mlpy
from matplotlib.mlab import PCA as mlabPCA

from itertools import tee, izip

drawingHandles = []
# TODO check the filenames before each run
groundtruth_filename = 'debug/PCA_groundtruth_idx.csv'
PCA_results_filename = 'debug/PCA_z_axis.csv'
trajs_path = 'trajs/Unscrew_Screw_LF_1_limits'
recovered_paths = 'trajs/Unscrew_recovered_4'
bags_file = "bags_csv/ScrewUnscrewWithoutHandArtem8/unscrew_nut_ScrewUnscrewWithoutHandArtem8.csv"
TSR_filename = 'Unscrew'


# Dynamic Time Warping
def my_dtw(s1, s2, distfn):
    n = len(s1)
    m = len(s2)
    cost = np.empty((n, m))
    cost[0][0] = distfn(s1[0], s2[0])
    for i in range(2, n):
        cost[i][0] = distfn(s1[i], s2[0]) + cost[i-1][0]
    for j in range(2, m):
        cost[0][j] = distfn(s1[0], s2[j]) + cost[0][j-1]
    for i in range(2, n):
        for j in range(2, m):
            cost[i][j] = distfn(s1[i], s2[j]) + min(
                    cost[i-1][j],
                    cost[i-1][j-1],
                    cost[i][j-1]
                    )
    return cost[n-1][m-1]


def dist_abs(x, y):
    return sum([abs(x[i]-y[i]) for i in range(len(x))])


def dist_euc(x, y):
    return sum([(x[i]-y[i])*(x[i]-y[i]) for i in range(len(x))])


def track_distance(t1, t2):
    t1, t2 = [y for (x,y) in t1], [y for (x,y) in t2]
#   x = mlpy.dtw_std(t1, t2, dist_only=True) # mlpy 1D DTW
    x = my_dtw(t1, t2, dist_abs)
    return x


def start_dtw():
    t1, t2 = all_tracks[0], all_tracks[1]
    for i,j in enumerate(all_tracks):
        for k,m in enumerate(all_tracks[i+1:]):
            dist = track_distance(all_tracks[i], all_tracks[i+k+1])
            print i, i+k+1, dist

# Class for drawing an error with matplotlib
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


def waitrobot(robot):
    q = robot.GetDOFValues()
    t = robot.GetTransform()
    robot.GetController().SetDesired(q, t)
    robot.WaitForController(0)


# Sliding window iterator
def sliding_window(iterable, size):
    iters = tee(iterable, size)
    for i in range(1, size):
        for each in iters[i:]:
            next(each, None)
    return izip(*iters)


# Read segmentation points from a file with groundtruth segmentation
def read_splits():
    splits = []
    with open(groundtruth_filename, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for row in reader:
            splits.append(int(row[0]))
    return splits


# Compare original and recovered trajectories by drawing them
def compare_trajs(robot, traj_orig, traj_rec):
    ee_points = np.zeros((0, 3))
    del drawingHandles[:]
    dt = 1/float(25)  # 25 Hz
    spec_points = np.zeros((0, 3))
    for j in range(traj_orig.GetNumWaypoints()):
        data = traj_orig.GetWaypoint(j)
        robot.SetDOFValues(data[:39])
        new_point = robot.GetActiveManipulator().GetEndEffectorTransform()[0:3, 3]
        ee_points = np.append(ee_points, [new_point], axis=0)
        asarray(ee_points)
        colors = []
        [colors.append((0, 0, 1)) for x in xrange(0, len(ee_points))]
        drawingHandles.append(env.plot3( points = asarray(ee_points), pointsize=3.5, colors = asarray(colors) ))
        waitrobot(robot)
        time.sleep(dt)
    print "Original trajectory is drawn"
    #print traj_rec.GetNumWaypoints()
    raw_input("Press ENTER to draw the recovered trajectory..")
    time.sleep(2)
    ee_points = np.zeros((0, 3))
    for j in range(traj_rec.GetNumWaypoints()):
        data = traj_rec.GetWaypoint(j)
        # extract the robot joint values only
        robot.SetDOFValues(data[:39])
        new_point = robot.GetActiveManipulator().GetEndEffectorTransform()[0:3, 3]
        ee_points = np.append(ee_points, [new_point], axis=0)
        asarray(ee_points)
        colors = []
        [colors.append((0, 1, 0)) for x in xrange(0, len(ee_points))]
        drawingHandles.append(env.plot3( points = asarray(ee_points), pointsize=3.5, colors = asarray(colors) ))
        waitrobot(robot)
        time.sleep(dt)


# Translates the trajectory (sequence of robot configurations) into end-effector points and transforms
def calculate_end_effector_poses(robot, traj_all):
    ee_points = np.zeros((0, 3))
    ee_trans = []
    del drawingHandles[:]
    dt = 1/float(25)  # 25 Hz
    for j in range(traj_all.GetNumWaypoints()):
        data = traj_all.GetWaypoint(j)
        # extract the robot joint values only
        robot.SetDOFValues(data[:39])
        new_point = robot.GetActiveManipulator().GetEndEffectorTransform()[0:3, 3]
        ee_points = np.append(ee_points, [new_point], axis=0)
        ee_trans.append(robot.GetActiveManipulator().GetEndEffectorTransform())
        asarray(ee_points)
        colors = []
        [colors.append((0, 0, 1)) for x in xrange(0, len(ee_points))]
        drawingHandles.append(env.plot3( points = asarray(ee_points), pointsize=3.0, colors = asarray(colors) ))
        waitrobot(robot)
        time.sleep(dt)
    return ee_points, ee_trans


# Remove duplicate points from trajectory (points when object captured with Vicon is not moving)
def remove_duplicates(points, trans):
    new_points = []
    new_trans = []
    if len(points) > 0:
        new_points.append(points[0])
        new_trans.append(trans[0])
        for i in range(1, len(points)):
            if not (points[i] == points[i-1]).all():
                new_points.append(points[i])
                new_trans.append(trans[i])
            else:
                print "point %r removed" % i
    return new_points, new_trans


# Resampling 3D curve (trajectory
def resampling_3D_curve(ee_points, ee_trans, num_samples = 200, verbose = False):
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
    resampled_trans = []
    resampled_trans.append(ee_trans[0])
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
        new_z = float(intp.ev(new_x, new_y))
        new_q_x = ee_trans[idx-1][0] + (ee_trans[idx][0] - ee_trans[idx-1][0]) * ratio
        new_q_y = ee_trans[idx-1][1] + (ee_trans[idx][1] - ee_trans[idx-1][1]) * ratio
        new_q_z = ee_trans[idx-1][2] + (ee_trans[idx][2] - ee_trans[idx-1][2]) * ratio
        new_q_w = ee_trans[idx-1][3] + (ee_trans[idx][3] - ee_trans[idx-1][3]) * ratio
        new_quat = np.array([new_q_x, new_q_y, new_q_z, new_q_w])
        new_point = np.array([new_x, new_y, new_z])
        resampled_curve.append(new_point)
        resampled_trans.append(new_quat)
    return resampled_curve[:], resampled_trans[:] # concat some points if needed


# check if two quaternions are close to each other
def close_orient(a, b, threshold = 0.2):
    dist = linalg.norm(a-b)
    #print dist, a
    if dist > threshold:
        return False
    return True


# normalize quaternion
def quat_normalize(q_raw):
    mag = np.sqrt(q_raw.dot(q_raw))
    return np.divide(q_raw,mag)


# Extract end-effector points and transforms from a rosbag file
def get_ee_poses_from_bags(robot, filename, draw_debug):
    draw_by_points = False
    ee_points = np.zeros((0, 3))
    ee_trans = []
    # varible that indicates if an object is holded by manipulator, or not. NOT USED IN A CURRENT VERSION OF CODE.
    holding = []
    with open(filename, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        # TODO make the code for filtering the transforms nice
        # code for filtering out noisy data from Vicon
        prev_orient = None
        idx = -1
        change_sign = False
        for row in reader:
            idx += 1
            # check if the two consequtive orientations are different
            quat = asarray([float(row[3]), float(row[4]), float(row[5]), float(row[6])])
            new_orient = quat
            if prev_orient == None:
                prev_orient = quat # first point
            else:
                if (not close_orient(new_orient, prev_orient)):
                    if (not close_orient(-1*(new_orient), prev_orient)):
                        continue
                    else:
                        change_sign = True
            if change_sign:
                quat = -1*quat
            prev_orient = quat
            ee_points = np.append(ee_points, [asarray([float(row[0]), float(row[1]), float(row[2])])], axis = 0)
            # store quaternions
            ee_trans.append(quat)
    # unform resampling of a trajectory points and transforms
    ee_points_res, ee_q_res = resampling_3D_curve(ee_points, ee_trans, num_samples=200)
    ee_points_res = array(ee_points_res)
    ee_trans_res =[]
    i = 0
    for q in ee_q_res:
        new_trans = BuildMatrix(ee_points_res[i].tolist(), q.tolist())
        ee_trans_res.append(new_trans)
        i += 1
    print "resampling done"
    if draw_debug:
        colors = []
        [colors.append((0, 0, 1)) for x in xrange(0, len(ee_points))]
        colors2 = []
        [colors2.append((0, 0,0 )) for x in xrange(0, len(ee_points_res))]
        drawingHandles.append(env.plot3( points = asarray(ee_points), pointsize=3.0, colors = asarray(colors) ))
        raw_input("Original trajectory is drawn. Press ENTER to draw resampled trajectory..")
        del drawingHandles[:]
        drawingHandles.append(env.plot3( points = asarray(ee_points_res), pointsize=3.5, colors = asarray(colors2) ))
        raw_input("Press ENTER. All points are drawn..")
    # handcoded holding. Not used
    for i in range(len(ee_points)):
        holding.append(1 if i < 42 else 0)
    # drawing one point at a time for debugging purposes
    if draw_debug and draw_by_points:
        del drawingHandles[:]
        colors = []
        colors.append((0.6, 0.6, 0.0))
        for i in range(len(ee_points_res)):
            drawingHandles.append(env.plot3( points = asarray(ee_points_res[i]), pointsize=3.0, colors = asarray(colors) ))
            print "point %r is drawn" %i
            print ee_points_res[i]
            sys.stdin.readline()
    return ee_points_res, ee_trans_res, holding


# Take a directory that contains list of OpenRave trajectories and return a single trajectory
# Also write the segmentation points into groundtruth
def process_trajs(env, traj_dir):
    traj_list = sorted(os.listdir(traj_dir))
    traj_orig = RaveCreateTrajectory( env, '')
    traj_orig.Init(robot.GetActiveConfigurationSpecification())
    open(groundtruth_filename, 'w').close()
    idx = 0
    for filename in traj_list:
        traj = RaveCreateTrajectory( env, '')
        full_path = traj_dir + '/' + filename
        f = open(full_path, 'r')
        traj.deserialize(f.read())
        with open(groundtruth_filename, 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            writer.writerow([idx])
        for i in range(traj.GetNumWaypoints()):
            traj_orig.Insert(idx, traj.GetWaypoint(i)[:39])
            idx += 1
    traj_list = sorted(os.listdir(recovered_paths))
    traj_rec = RaveCreateTrajectory( env, '')
    traj_rec.Init(robot.GetActiveConfigurationSpecification())
    # optional code to test already recovered trajectories
    idx = 0
    for filename in traj_list:
        traj = RaveCreateTrajectory( env, '')
        full_path = recovered_paths + '/' + filename
        f = open(full_path, 'r')
        traj.deserialize(f.read())
        for i in range(traj.GetNumWaypoints()):
            traj_rec.Insert(idx, traj.GetWaypoint(i)[:39])
            idx += 1
    return traj_orig, traj_rec


# Principal component analysis, save results in TSR-format
def run_PCA(window_points, ee_trans, hold, stud_offset=[], plot = False, draw_axes = False, verbose = False, eigenvalue_threshold = 0.001):

    # 1. PCA calculations
        mean_x = np.mean(window_points[:, 0])
        mean_y = np.mean(window_points[:, 1])
        mean_z = np.mean(window_points[:, 2])
        mean_vector = np.array([[mean_x], [mean_y], [mean_z]])
        if verbose: print 'Mean Vector:\n', mean_vector
        dim = 3
        scatter_matrix = np.zeros((dim, dim))
        for i in range(window_points.shape[0]):
            scatter_matrix += (window_points[i, :].reshape(dim, 1)
                - mean_vector).dot((window_points[i, :].reshape(dim, 1) - mean_vector).T)
        if verbose: print 'Scatter Matrix:\n', scatter_matrix
        cov_mat = np.cov([window_points[:, 0], window_points[:, 1], window_points[:, 2]])
        if verbose: print 'Covariance Matrix:\n', cov_mat
        # eigenvectors and eigenvalues for the from the scatter matrix
        eig_val_sc, eig_vec_sc = np.linalg.eigh(scatter_matrix)
        # eigenvectors and eigenvalues for the from the covariance matrix
        eig_val_cov, eig_vec_cov = np.linalg.eig(cov_mat)
        for i in range(len(eig_val_sc)):
            eigvec_sc = eig_vec_sc[:, i]#.reshape(1, 3).T
            eigvec_cov = eig_vec_cov[:, i]#.reshape(1, 3).T
            assert eigvec_sc.all() == eigvec_cov.all(), 'Eigenvectors are not identical'
            if verbose: print 'Eigenvector {}: \n{}'.format(i+1, eig_vec_sc[:, i])
            if verbose: print 'Eigenvalue {} from scatter matrix: {}'.format(i+1, eig_val_sc[i])
            if verbose: print 'Eigenvalue {} from covariance matrix: {}'.format(i+1, eig_val_cov[i])
            if verbose: print 'Scaling factor: ', eig_val_sc[i]/eig_val_cov[i]
            if verbose: print 40 * '-'
        # check eigenvector-eigenvalue calculations
        for i in range(len(eig_val_sc)):
            eigv = eig_vec_sc[:, i].reshape(1, dim).T
            np.testing.assert_array_almost_equal(scatter_matrix.dot(eigv),
                    eig_val_sc[i] * eigv, decimal=6,
                    err_msg='', verbose=True)
        if plot:
            fig = plt.figure(1)
            #fig = plt.figure(figsize=(7, 7))
            ax = fig.add_subplot(111, projection='3d')

            plt.plot(window_points[:, 0], window_points[:, 1],
                window_points[:, 2],
                'o', markersize=8, color='green', alpha=0.2)
            plt.plot([mean_x], [mean_y],
                    [mean_z],
                'o', markersize=10, color='red', alpha=0.5)
            first = True
            for v in eig_vec_sc.T:
                c = 'b'
                if first:
                    c = 'r'
                    first = False
                a = Arrow3D([window_points[0, 0], v[0]+window_points[0, 0]], [window_points[0, 1], v[1]+window_points[0, 1]],
                    [window_points[0, 2], v[2]+window_points[0, 2]],
                    mutation_scale=1, lw=3, arrowstyle="-|>", color=c)
                ax.add_artist(a)
            ax.set_xlabel('x_values')
            ax.set_ylabel('y_values')
            ax.set_zlabel('z_values')
            plt.title('Eigenvectors')
            plt.draw()
        # test if the eigenvectors are unit vectors
        # for ev in eig_vec_sc:
        #     np.testing.assert_array_almost_equal(1.0, np.linalg.norm(ev))
        # Make a list of (eigenvalue, eigenvector) tuples
        eig_pairs = [(np.abs(eig_val_sc[i]), eig_vec_sc[:, i]) for i in range(len(eig_val_sc))]
        # Sort the (eigenvalue, eigenvector) tuples from high to low
        eig_pairs.sort()
        eig_pairs.reverse()
        matrix_w = np.hstack((eig_pairs[0][1].reshape(3, 1), eig_pairs[1][1].reshape(3, 1), eig_pairs[2][1].reshape(3, 1)))
        if verbose: print 'New coordinates:\n', matrix_w
        transformed = matrix_w.T.dot(window_points.T)
        if plot:
            fig = plt.figure(2)
            ax = fig.add_subplot(111, projection='3d')
            plt.plot(transformed.T[:, 0], transformed.T[:, 1], transformed.T[:, 2],
                 'o', markersize=7, color='blue', alpha=0.5)
            ax.set_xlabel('x_values')
            ax.set_ylabel('y_values')
            ax.set_zlabel('z_values')
            plt.title('Transformed samples with class labels')
            plt.draw()

    # 2. TSR calculations from PCA results
        # Path TSR frame
        TSR_0_w_path = MakeTransform(matrix_w, matrix(window_points[0, :]))
        change_axes = MakeTransform(rodrigues([0, pi/2, 0]), matrix([0, 0, 0]))*MakeTransform(rodrigues([0, 0, pi/2]), matrix([0, 0, 0]))
        TSR_0_w_path = TSR_0_w_path*change_axes
        # check for valid rotation matrix (det = 1) and fix
        if numpy.linalg.det(TSR_0_w_path[0:3][:,0:3]) != 1:
            TSR_0_w_path[0:3, 0] *= -1;
        # Path  EE offset
        EE_offset_path = MakeTransform(eye(3), matrix([0, 0, 0]))
        # Path B_w
        if verbose:
            print "eigen pairs"
            print eig_pairs[0][0]
            print eig_pairs[1][0]
            print eig_pairs[2][0]
        # print eigenvalue_threshold
        # TODO fix the frames from extraction to execution
        z_min = np.min(transformed.T[:, 0])-transformed[0, 0]-0. if (abs(eig_pairs[0][0]) < eigenvalue_threshold) else -1000
        z_max = np.max(transformed.T[:, 0])-transformed[0, 0]+0. if (abs(eig_pairs[0][0]) < eigenvalue_threshold) else 1000
        x_min = np.min(transformed.T[:, 1])-transformed[1, 0]-0. if (abs(eig_pairs[1][0]) < eigenvalue_threshold) else -1000
        x_max = np.max(transformed.T[:, 1])-transformed[1, 0]+0. if (abs(eig_pairs[1][0]) < eigenvalue_threshold) else 1000
        y_min = np.min(transformed.T[:, 2])-transformed[2, 0]-0. if (abs(eig_pairs[2][0]) < eigenvalue_threshold) else -1000
        y_max = np.max(transformed.T[:, 2])-transformed[2, 0]+0. if (abs(eig_pairs[2][0]) < eigenvalue_threshold) else 1000
        B_w_path = mat([x_min, x_max, y_min, y_max, z_min, z_max, -1000, 1000, -1000, 1000, -1000, 1000])
        if verbose:
            print "boundaries:"
            print transformed
            print B_w_path
        # Goal TSR frame
        #TSR_0_w_goal = ee_trans[-1] # not gonna be used
        TSR_0_w_goal = ee_trans[0]
        # Goal EE offset
        #EE_offset_goal = MakeTransform(eye(3), matrix([0, 0, 0]))
        EE_offset_goal = ee_trans[-1] #MakeTransform(eye(3), matrix([0, -0.05, -0.002])) # for the unscrew nut task (took from AppraochAction)
        #EE_offset_goal = stud_offset[-1] # for the screw task the offset
        # Goal B_w
        B_w_goal = mat([-0, 0, -0, 0, -0, 0, -0, 0, -0, 0, -0, 0])
        if draw_axes:
            #draw TSR_frame
            #drawingHandles.append(misc.DrawAxes(env, TSR_0_w_path, 0.035, linewidth=1.0))
            drawingHandles.append(misc.DrawAxes(env, TSR_0_w_path, 0.15, linewidth=1.0))
            #drawingHandles.append(misc.DrawAxes(env, TSR_0_w_goal, 0.3))
        new_z_axis = eig_pairs[0][1]
        plt.show(block = False)

        return new_z_axis, [TSR_0_w_path, EE_offset_path, B_w_path], [TSR_0_w_goal, EE_offset_goal, B_w_goal]


# Create straing line from start and end points for debugging purposes
def create_st_line(points):
    print "first point", points[0]
    print "last points", points[-1]
    #sys.stdin.readline()
    incr_x = (points[-1][0] - points[0][0])/len(points)
    incr_y = (points[-1][1] - points[0][1])/len(points)
    incr_z = (points[-1][2] - points[0][2])/len(points)
    for i in range(1, len(points)-1):
        points[i] = points[0] + i * np.array([incr_x, incr_y, incr_z])
    return points


if __name__ == "__main__":
    # set this variable to True if you extract from a rosbag file,
    # otherwise the extraction will be performed from OpenRave trajectories
    extract_from_bags = True
    print "Setting up the environment..."
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('robots/pr2-beta-static.zae')
    robot = env.GetRobots()[0]
    robot.SetActiveManipulator("rightarm")
    # move robot to see the trajectory points
    T = robot.GetTransform()
    T[0,3] -= 2
    robot.SetTransform(T)
    time.sleep(0.1)
    # set to True if you want to plot results of segmentation
    to_plot = False
    if not extract_from_bags:
        print "Extract from trajectories"
        print "Process the trajectories..."
        traj_all, traj_rec = process_trajs(env, traj_dir = trajs_path)
        if False:
            raw_input("Press ENTER to compare trajectories..")
            compare_trajs(robot, traj_all, traj_rec)
        raw_input("Press ENTER to display all trajectories..")
        ee_points, ee_trans = calculate_end_effector_poses(robot, traj_all)
        raw_input("Press ENTER to run PCA...")
        groundtruth_splits = read_splits()
    else:
        print "Extract from bags"
        raw_input("Press ENTER to display the bags..")
        ee_points, ee_trans, holding = get_ee_poses_from_bags(robot, bags_file, True)

    for window_size in [10]:
        z_axis_windowed = []
        open(PCA_results_filename, 'w').close()
        print "Compute segmentation for window size of " + str(window_size)
        print "number of ee points", len(ee_points)
        p_const = [] # variable to keep TSR for comparison
        # run Pca in a sliding window and calculate 1st principal components (z-axis_windowed)
        for each in sliding_window(ee_points, window_size):
            window_points = asarray(each)
            z_axis, path_constr, goal_constr = run_PCA(window_points, ee_trans, holding, stud_offset = [0], plot = False, draw_axes = False, verbose = False)
            p_const.append(path_constr)
            z_axis_windowed.append(z_axis)
            with open(PCA_results_filename, 'a') as csvfile:
                writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(z_axis)
        raw_input("Press ENTER to go further..")

        # dot products of a sliding window of consequtive z-vectors
        dot_products = np.zeros((len(z_axis_windowed), 7))
        for i in range(3, len(z_axis_windowed)-3):
            dot_products[i][0] = z_axis_windowed[i][:].dot(z_axis_windowed[i-3][:])
            dot_products[i][1] = z_axis_windowed[i][:].dot(z_axis_windowed[i-2][:])
            dot_products[i][2] = z_axis_windowed[i][:].dot(z_axis_windowed[i-1][:])
            dot_products[i][3] = z_axis_windowed[i][:].dot(z_axis_windowed[i+1][:])
            dot_products[i][4] = z_axis_windowed[i][:].dot(z_axis_windowed[i+2][:])
            dot_products[i][5] = z_axis_windowed[i][:].dot(z_axis_windowed[i+3][:])
            dot_products[i][6] = (abs(dot_products[i, 1]) + abs(dot_products[i, 2]) + abs(dot_products[i, 2])
                + abs(dot_products[i, 3]) + abs(dot_products[i, 4]) + abs(dot_products[i, 5]) )/6

        # remove 3 first and last zero points (to scale the graph)
        dot_products = dot_products[3:(len(dot_products)-3)]
        if True:
            plt.figure(window_size)
            plt.plot(range(len(dot_products)), dot_products[:, 6], color='blue', alpha=0.5)
            if not extract_from_bags:
                for split in groundtruth_splits:
                    plt.plot([split, split], [0, 1], '-', color='red', alpha=0.5)
            plt.xlabel('trajectory waypoints', fontsize = 28)
            plt.ylabel('z axis dot products', fontsize = 28)
            plt.title('Segmentation results for window of size ' + str(window_size), fontsize = 40)
            plt.show(block=False)
        raw_input("Press ENTER to calculate changepoints..")
        # inverse the dot_product sequence (to calculate peaks, rather than rims
        neg_dot_prod_z = [-i for i in dot_products[:, 6]]

        # find peaks
        peakind = scipy.signal.find_peaks_cwt(neg_dot_prod_z, np.arange(1, 10))
        print peakind, dot_products[peakind] # you can add 3 points back here to compensate for window
        # threshold for segmentation point detection
        tresholdind = []
        threshold = 0.995
        for ind in peakind:
            if dot_products[ind, 6] < threshold:
                tresholdind.append(ind)
        # find minimum point in a neighborhood
        minind = []
        for ind in tresholdind:
            val, idx = min((abs(val), idx) for (idx, val) in enumerate(dot_products[ind, 0:6]))
            if idx > 2:
                minind.append(ind+idx-2)
            else:
                minind.append(ind+idx-3)
        final_cpoints = [i+3 for i in minind] # compensate for cut of 0's in dot_products
        change_points = final_cpoints

        raw_input("Press ENTER to write the TSR's into file..")
        last_ind = 0
        open(TSR_filename, 'w').close()
        list_of_actions = []
        # print ee_points[0]
        for i in change_points:
            print i
            current_points = asarray(ee_points[last_ind:i])
            current_transf = ee_trans[last_ind:i]
            current_holding = holding[last_ind:i]
            last_ind = i
            sys.stdin.readline()
            z_axis, path_constr, goal_constr = run_PCA(current_points, current_transf, current_holding, stud_offset = [0], plot = False, draw_axes = True, verbose = True)
            raw_input("Press enter for the next segment...")
            # second argument are the manipulator indecies
            list_of_actions.append(["R", [5, 7], True, path_constr[0], path_constr[1], path_constr[2],
                                   goal_constr[0], goal_constr[1], goal_constr[2]])
        pickle.dump(list_of_actions, open(TSR_filename, "wb"))

    raw_input("Press enter to exit...")

