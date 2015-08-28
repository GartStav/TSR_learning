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

from matplotlib.mlab import PCA as mlabPCA

from wpi_planning_utilities.transformation_helper import *

# drawing
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch
from wpi_planning_utilities.TransformMatrix import *
from wpi_planning_utilities.rodrigues import *
from scipy.spatial import ConvexHull

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

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
bag_name  = "ScrewUnscrewDemonstrationsArtem1"
# bags_file = "bags_csv/" + bag_name + "/nut_" + bag_name + ".csv"
#bags_file = "bags_csv/ScrewUnscrewWithoutHandArtem8/screw_nut_ScrewUnscrewWithoutHandArtem8.csv"
bags_file = "bags_csv/UnhangHangWithoutHandArtem1/hang_wheel_UnhangHangWithoutHandArtem1.csv"

draw_by_points = False

TSR_filename = 'Hang'
#TSR_filename = 'TSR_segmentation/Rec_Task_2'

import numpy as np

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

def sliding_window(iterable, size):

    iters = tee(iterable, size)
    for i in range(1, size):
        for each in iters[i:]:
            next(each, None)
    return izip(*iters)

def read_splits():
    splits = []
    with open(groundtruth_filename, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for row in reader:
            splits.append(int(row[0]))

    return splits

# compare original and recovered trajectories by drawing them
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

        # some stupid hack
        # if (j > 112) & (j <159):
        #     spec_points = np.append(spec_points, [new_point], axis=0)

        waitrobot(robot)
        time.sleep(dt)

    # some stupid hack
    # colors = []
    # [colors.append((0, 1, 0)) for x in xrange(0, len(spec_points))]
    # drawingHandles.append(env.plot3( points = asarray(spec_points), pointsize=6.5, colors = asarray(colors) ))


    print "Original trajectory is drawn"
    print traj_rec.GetNumWaypoints()
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



def calculate_end_effector_poses(robot, traj_all):

    ee_points = np.zeros((0, 3))
    ee_trans = []
    del drawingHandles[:]
    dt = 1/float(25)  # 25 Hz

    for j in range(traj_all.GetNumWaypoints()):
        data = traj_all.GetWaypoint(j)
        # extract the robot joint values only
        robot.SetDOFValues(data[:39])

        #drawingHandles.append(misc.DrawAxes(env, robot.GetActiveManipulator().GetEndEffectorTransform(), 1))
        new_point = robot.GetActiveManipulator().GetEndEffectorTransform()[0:3, 3]
        # with open('EE_positions_'+str(counter)+'.csv', 'a') as csvfile:
        #     writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        #     writer.writerow(new_point)

        ee_points = np.append(ee_points, [new_point], axis=0)
        ee_trans.append(robot.GetActiveManipulator().GetEndEffectorTransform())

        print new_point
        print type(new_point)
        print type(new_point[0])

        sys.stdin.readline()

        asarray(ee_points)

        colors = []
        [colors.append((0, 0, 1)) for x in xrange(0, len(ee_points))]

        drawingHandles.append(env.plot3( points = asarray(ee_points), pointsize=3.0, colors = asarray(colors) ))

        waitrobot(robot)
        time.sleep(dt)

    return ee_points, ee_trans

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
    return resampled_curve[:], resampled_trans[:] # concat some points

def close_orient(a, b, threshold = 0.2):
    dist = linalg.norm(a-b)
    print dist, a
    if dist > threshold:
        return False
    return True

def quat_normalize(q_raw):
    mag = np.sqrt(q_raw.dot(q_raw))
    #print mag
    return np.divide(q_raw,mag)

def get_ee_poses_from_bags(robot, filename):

    ee_points = np.zeros((0, 3))
    ee_trans = []
    holding = []

    with open(filename, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        # TODO make the code for filtering the transforms nice
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
                        #print "bad orient at idx = ", idx
                        continue
                    else:
                        change_sign = True
            if change_sign:
                quat = -1*quat
            prev_orient = quat
            ee_points = np.append(ee_points, [asarray([float(row[0]), float(row[1]), float(row[2])])], axis = 0)
            #new_trans = BuildMatrix([float(row[i]) for i in range(3)], [float(row[i]) for i in range(3,len(row))])
            # store quaternions
            ee_trans.append(quat)

    print len(ee_points)
    print len(ee_trans)


    # remove the noisy point TODO hack
    #ee_points = delete(ee_points, 10, 0)
    #del ee_trans[10]

    # cut out the points in the end
    # ee_points = ee_points[:20]
    # ee_trans = ee_trans[:20]

    #print ee_trans[1]

    # sys.stdin.readline()
    #
    # raw_input("Draw transforms")
    # i = 0
    # for t in ee_trans:
    #     new_trans = BuildMatrix(ee_points[i].tolist(), t.tolist())
    #     drawingHandles.append(misc.DrawAxes(env, new_trans, 0.3))
    #     #print i
    #     i += 1
    #     #sys.stdin.readline()
    # raw_input("Transforms are drawn")

    #crearte straight line for constrained path
    # straight_line = create_st_line(ee_points[114:137])
    # ee_points[114:137] = straight_line

    # write results into csv file
    # open('nut_ArtemUnscrewScrewNut-1_straight.csv', 'w').close()
    # with open('nut_ArtemUnscrewScrewNut-1_straight.csv', 'a') as csvfile:
    #     writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    #     for e in ee_points:
    #         writer.writerow(e)


    #ee_points = ee_points[114:192]

    # SEGMENT OUT UNSCREW FROM SCREW - BAG 2 [0:48], [48:]
    # ee_points, ee_trans = remove_duplicates(ee_points[:], ee_trans[:])
    #
    # print len(ee_points)

    ee_points_res, ee_q_res = resampling_3D_curve(ee_points, ee_trans, num_samples=200)
    ee_points_res = array(ee_points_res)

    print len(ee_points_res)

    ee_trans_res =[]
    i = 0
    for q in ee_q_res:
        new_trans = BuildMatrix(ee_points_res[i].tolist(), q.tolist())
        ee_trans_res.append(new_trans)
        i += 1

    print "resampling done"
    sys.stdin.readline()

    # raw_input("Draw transforms")
    # i = 0
    # for t in ee_trans_res:
    #     drawingHandles.append(misc.DrawAxes(env, t, 0.3))
    #     #print i
    #     i += 1
    #     #sys.stdin.readline()
    # raw_input("Transforms are drawn")


    # index  = 0
    # for point in ee_points:
    #     print point
    #     print index
    #     index += 1
    #     drawingHandles.append(env.plot3( points = asarray(point), pointsize=3.0, colors = asarray([(0, 0, 1)]) ))
    #     sys.stdin.readline()

    colors = []
    [colors.append((0, 0, 1)) for x in xrange(0, len(ee_points))]

    colors2 = []
    [colors2.append((0, 0,0 )) for x in xrange(0, len(ee_points_res))]

    drawingHandles.append(env.plot3( points = asarray(ee_points), pointsize=3.0, colors = asarray(colors) ))

    print "draw resampled"

    sys.stdin.readline()
    del drawingHandles[:]

    drawingHandles.append(env.plot3( points = asarray(ee_points_res), pointsize=3.5, colors = asarray(colors2) ))

    print len(ee_points)

    raw_input("Press ENTER. Points are drawn..")

    for i in range(len(ee_points)):
        holding.append(1 if i < 42 else 0)

    if draw_by_points:
        del drawingHandles[:]
        colors = []
        colors.append((0.6, 0.6, 0.0))
        for i in range(len(ee_points_res)):
            drawingHandles.append(env.plot3( points = asarray(ee_points_res[i]), pointsize=3.0, colors = asarray(colors) ))
            print "point %r is drawn" %i
            print ee_points_res[i]
            sys.stdin.readline()

    #drawingHandles.append(env.plot3( points = asarray(ee_points), pointsize=3.0, colors = asarray(colors) ))

    #[20, 53, 73, 82, 93, len(dot_products)] # 5 segment is constrained, unscrew segment not detected #114
    #[20, 37, 67, 85, 96, len(dot_products)] # 2nd segment and 5 segment are constrained #115
    #[20, 34, 66, 77, 83, len(dot_products)] # 2nd and 5 segments are constrained #97
    #[21, 36, 76, 87, 94, len(dot_products)] # 2nd and 5 segments are constrained #116

    if False:

        colors1 = []
        [colors1.append((1, 0, 0)) for x in xrange(0, len(ee_points))]

        drawingHandles.append(env.plot3( points = asarray(ee_points[0:21]), pointsize=3.0, colors = asarray(colors1[0:21]) ))

        raw_input("Press ENTER. first segment is drawn..")

        colors2 = []
        [colors2.append((0, 0, 1)) for x in xrange(0, len(ee_points))]

        drawingHandles.append(env.plot3( points = asarray(ee_points[21:38]), pointsize=3.0, colors = asarray(colors2[21:38]) ))

        raw_input("Press ENTER. second segment is drawn..")

        colors3 = []
        [colors3.append((0, 0.4, 0)) for x in xrange(0, len(ee_points))]

        drawingHandles.append(env.plot3( points = asarray(ee_points[36:48]), pointsize=3.0, colors = asarray(colors3[36:48]) ))

        raw_input("Press ENTER. third segment is drawn..")

        # colors4 = []
        # [colors4.append((0.25, 0, 0.4)) for x in xrange(0, len(ee_points))]
        #
        # drawingHandles.append(env.plot3( points = asarray(ee_points[76:87]), pointsize=3.0, colors = asarray(colors4[76:87]) ))
        #
        # raw_input("Press ENTER. forth segment is drawn..")
        #
        # colors5 = []
        # [colors5.append((0.6, 0.6, 0.0)) for x in xrange(0, len(ee_points))]
        #
        # drawingHandles.append(env.plot3( points = asarray(ee_points[87:94]), pointsize=3.0, colors = asarray(colors5[87:94]) ))
        #
        # raw_input("Press ENTER. fifth segment is drawn..")
        #
        # colors6 = []
        # [colors6.append((0.0, 0.0, 0.0)) for x in xrange(0, len(ee_points))]
        #
        # drawingHandles.append(env.plot3( points = asarray(ee_points[94:len(ee_points)]), pointsize=3.0, colors = asarray(colors6[94:len(ee_points)]) ))
        #
        # raw_input("Press ENTER. sixth segment is drawn..")

    return ee_points_res, ee_trans_res, holding

def process_trajs(env, traj_dir):

    traj_list = sorted(os.listdir(traj_dir))
    traj_orig = RaveCreateTrajectory( env, '')
    traj_orig.Init(robot.GetActiveConfigurationSpecification())

    open(groundtruth_filename, 'w').close()

    idx = 0
    for filename in traj_list:
        #counter += 1
        traj = RaveCreateTrajectory( env, '')
        full_path = traj_dir + '/' + filename
        f = open(full_path, 'r')
        traj.deserialize(f.read())
        with open(groundtruth_filename, 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            writer.writerow([idx])
        for i in range(traj.GetNumWaypoints()):
            #traj_all.append(traj.GetWaypoint())
            traj_orig.Insert(idx, traj.GetWaypoint(i)[:39])
            idx += 1

    traj_list = sorted(os.listdir(recovered_paths))
    traj_rec = RaveCreateTrajectory( env, '')
    traj_rec.Init(robot.GetActiveConfigurationSpecification())
    #print traj_list
    #sys.stdin.readline()

    idx = 0
    for filename in traj_list:
        #counter += 1
        traj = RaveCreateTrajectory( env, '')
        full_path = recovered_paths + '/' + filename
        f = open(full_path, 'r')
        traj.deserialize(f.read())

        for i in range(traj.GetNumWaypoints()):
            traj_rec.Insert(idx, traj.GetWaypoint(i)[:39])
            idx += 1

    return traj_orig, traj_rec

def Run_Mean_Shift():

    # Mean Shift, does not really work
    raw_input("Press ENTER to detect change points..")
    # model = MeanShiftModel()
    # stats_ts, pvals, nums = model.detect_mean_shift(dot_products, B=1000)
    # print stats_ts
    # plt.plot(range(len(stats_ts)), stats_ts)
    # plt.show()
    # print pvals
    # plt.plot(range(len(pvals)), pvals)
    # plt.show()
    # print nums
    # plt.plot(range(len(nums)), nums)
    # plt.show()

def run_PCA(window_points, ee_trans, hold, stud_offset=[], plot = False, draw_axes = False, verbose = False, eigenvalue_threshold = 0.001):

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

            # draw 2D axes
            # plt.plot([0+mean_x, eig_vec_sc[0, 0]+mean_x], [0+mean_y, eig_vec_sc[1, 0]+mean_y], 'r-', lw=2)
            # plt.plot([0+mean_x, eig_vec_sc[0, 1]+mean_x], [0+mean_y, eig_vec_sc[1, 1]+mean_y], 'k-', lw=2)

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

            #plt.show(block=False)
            plt.draw()

        #sys.stdin.readline()

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
            #fig = plt.figure(figsize=(7, 7))
            ax = fig.add_subplot(111, projection='3d')

            plt.plot(transformed.T[:, 0], transformed.T[:, 1], transformed.T[:, 2],
                 'o', markersize=7, color='blue', alpha=0.5)


            ax.set_xlabel('x_values')
            ax.set_ylabel('y_values')
            ax.set_zlabel('z_values')
            plt.title('Transformed samples with class labels')

            #plt.show(block=False)
            plt.draw()

        # Path TSR frame
        TSR_0_w_path = MakeTransform(matrix_w, matrix(window_points[0, :]))

        #sys.stdin.readline()

        change_axes = MakeTransform(rodrigues([0, pi/2, 0]), matrix([0, 0, 0]))*MakeTransform(rodrigues([0, 0, pi/2]), matrix([0, 0, 0]))
        TSR_0_w_path = TSR_0_w_path*change_axes

        # TODO check for valid rotation matrix (det = 1) and fix
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
        #print window_points[0, :]
        if verbose:
            print "boundaries:"
            print transformed
            print B_w_path
        #sys.stdin.readline()

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

def create_st_line(points):
    print "first point", points[0]
    print "last points", points[-1]
    #sys.stdin.readline()
    incr_x = (points[-1][0] - points[0][0])/len(points)
    incr_y = (points[-1][1] - points[0][1])/len(points)
    incr_z = (points[-1][2] - points[0][2])/len(points)
    for i in range(1, len(points)-1):
        points[i] = points[0] + i * np.array([incr_x, incr_y, incr_z])
        #print points[i]
    return points


if __name__ == "__main__":

    extract_from_bags = True

    # x = [1,2,3,4,5,6,7,8,9,10]
    # y = [0,1,2,3,0,4,0,5,6,7,0,8,9,0,10,0,0]
    # dist, cost, path = mlpy.dtw_std(x, y, dist_only=False)
    #
    # print "dist", dist
    # print "cost", cost
    # print "path", path

    # points = np.random.rand(8, 3)   # 30 random points in 2-D
    # hull = ConvexHull(points)
    #
    # plt.plot(points[:,0], points[:,1], 'o')
    # for simplex in hull.simplices:
    #     plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
    #
    # plt.show()
    # sys.stdin.readline()

    print "Setting up the environment..."
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('robots/pr2-beta-static.zae')
    robot = env.GetRobots()[0]
    robot.SetActiveManipulator("rightarm")

    T = robot.GetTransform()
    T[0,3] -= 2
    robot.SetTransform(T)

    time.sleep(0.1)
        # Load the plugin
    # prob = RaveCreateProblem(env, 'bridge')
    # env.LoadProblem(prob, "")
    #
    # if prob:
    #     print "plugin loaded"
    # sys.stdin.readline()

    to_plot = True

    #ee_points, ee_trans = get_ee_poses_from_bags(robot, "EE_from_bags_3.csv")
    #ee_points2, ee_trans = get_ee_poses_from_bags(robot, "EE_from_bags_2.csv")

    #dist, cost, path = mlpy.dtw_std(ee_points1, ee_points2, dist_only=False)

    # print "dist", dist
    # print "cost", cost
    # print "path", path

    # sys.stdin.readline()

    if not extract_from_bags:

        print "Extract from trajectories"

        print "Process the trajectories..."
        traj_all, traj_rec = process_trajs(env, traj_dir = trajs_path)

        if False:
            raw_input("Press ENTER to compare trajectories..")
            compare_trajs(robot, traj_all, traj_rec)

        raw_input("Press ENTER to display all trajectories..")
        ee_points, ee_trans = calculate_end_effector_poses(robot, traj_all)

        # experimental code

        #z_axis, path_constr, goal_constr = run_PCA(ee_points[0:60, :], ee_trans[0:77], plot = True, draw_axes = True, verbose = True)
        #z_axis, path_constr, goal_constr = run_PCA(ee_points[77:88, :], ee_trans, plot = True, draw_axes = True, verbose = True)
        #z_axis, path_constr, goal_constr = run_PCA(ee_points[88:141, :], ee_trans, plot = False, draw_axes = True, verbose = True)
        #z_axis, path_constr, goal_constr = run_PCA(ee_points[141:149, :], ee_trans, plot = False, draw_axes = True, verbose = True)
        #z_axis, path_constr, goal_constr = run_PCA(ee_points[149:-1, :], ee_trans, plot = False, draw_axes = True, verbose = True)

        #raw_input("End of the experimental code..")

        # end of the experimental code

        raw_input("Press ENTER to run PCA...")

        groundtruth_splits = read_splits()

    else:

        print "Extract from bags"

        raw_input("Press ENTER to display the bags..")
        ee_points, ee_trans, holding = get_ee_poses_from_bags(robot, bags_file)
        #stud_points, stud_trans, holding = get_ee_poses_from_bags(robot, "stud_from_bags_2.csv")

        # take part of the original trajectory - unscrew
        #ee_points = ee_points[114:192]
        #ee_trans = ee_trans[114:192]


    # raw_input("Draw transforms")
    # for t in ee_trans:
    #     drawingHandles.append(misc.DrawAxes(env, t, 0.3))
    # raw_input("Transforms are drawn")


    #segment on hold and not hold parts
    #ee_points = ee_points[:42]

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

            z_axis, path_constr, goal_constr = run_PCA(window_points, ee_trans, holding, stud_offset = [0], plot = False, draw_axes = False, verbose = False)

            p_const.append(path_constr)

            z_axis_windowed.append(z_axis)

            with open(PCA_results_filename, 'a') as csvfile:
                writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(z_axis)

            #print "new step of sliding window" #change point in 21 for resampled straight line, 24 for resampled noisy traj
        sys.stdin.readline()

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
            plt.xlabel('trajectory waypoints', fontsize = 28)
            plt.ylabel('z axis dot products', fontsize = 28)
            plt.title('Segmentation results for window of size ' + str(window_size), fontsize = 40)
            plt.show(block=False)

        raw_input("Press ENTER to calculate changepoints..")

        neg_dot_prod_z = [-i for i in dot_products[:, 6]]

        # if to_plot:
        #     plt.plot(range(len(dot_products)), neg_dot_prod_z, color='red', alpha=0.5)
        #     plt.show()

        print "dot products are:"
        print dot_products[:, 6]
        sys.stdin.readline()
        print "negations of dot products are:"
        print neg_dot_prod_z
        sys.stdin.readline()
        peakind = scipy.signal.find_peaks_cwt(neg_dot_prod_z, np.arange(1, 10))
        #shifted_idx = [i+3 for i in peakind]
        #print shifted_idx
        print peakind, dot_products[peakind] # add 3 to compensate for window
        tresholdind = []
        threshold = 0.9982
        for ind in peakind:
            if dot_products[ind, 6] < threshold:
                tresholdind.append(ind)
        print tresholdind, dot_products[tresholdind]
        minind = []
        for ind in tresholdind:
            val, idx = min((abs(val), idx) for (idx, val) in enumerate(dot_products[ind, 0:6]))
            if idx > 2:
                minind.append(ind+idx-2)
            else:
                minind.append(ind+idx-3)

        print minind, dot_products[minind]

        final_cpoints = [i+3 for i in minind] # compensate for cut of 0's in dot_products

        print final_cpoints

        sys.stdin.readline()




        #change_points = [20, 53, 73, 82, len(dot_products)] #unscrew 1
        #change_points = [20, 37, 67, 85, 96, len(dot_products)] #unscrew 2
        #change_points = [20, 34, 66, 77, 83, len(dot_products)] #unscrew 3
        #change_points = [14, 21, 36, 76, 87, len(dot_products)] #unscrew 4
        #change_points = [16, 30, 37, 50, 112, len(dot_products)] # unscrew-ScrewUnscrewDemonstrationsArtem2
        #change_points = [143, 150, 165, 171, len(ee_points)] # screw-ScrewUnscrewDemonstrationsArtem2

        #change_points = [19, len(ee_points)] #unhang_wheel_UnhangHangWithoutHandArtem1
        #change_points = [44, 60, 69, len(ee_points)] #unhang_wheel_UnhangHangWithoutHandArtem2
        #change_points = [48, 58, 151, len(ee_points)] #unhang_wheel_UnhangHangWithoutHandArtem3
        #change_points = [108, 127, 154, 162, 181, len(ee_points)] #hang_wheel_UnhangHangWithoutHandArtem2
        change_points = [178, len(ee_points)] #hang_wheel_UnhangHangWithoutHandArtem2
        #change_points = [183, len(ee_points)] #hang_wheel_UnhangHangWithoutHandArtem2

        #final_cpoints.append(len(ee_points))
        #change_points = final_cpoints #unhang_wheel_UnhangHangWithoutHandArtem3


        #change_points = [49, 100] #len(dot_products)] # unscrew-ScrewUnscrewWithoutHandArtem8
        #change_points = [134, len(dot_products)] #len(dot_products)] # screw-ScrewUnscrewWithoutHandArtem8
        #change_points = [100] #len(dot_products)] # screw-ScrewUnscrewWithoutHandArtem8

        #change_points = [112, 146, 159, 189, 207, 220, 286, 329, 372, 427, 536, len(dot_products)] #unhang
        print change_points

        raw_input("Press ENTER to write the TSR's into file..")

        last_ind = 0
        open(TSR_filename, 'w').close()

        list_of_actions = []

        print ee_points[0]

        for i in change_points:
            print i
            current_points = asarray(ee_points[last_ind:i])
            current_transf = ee_trans[last_ind:i]
            current_holding = holding[last_ind:i]
            #current_stud = stud_trans[last_ind:i]
            last_ind = i
            print current_points
            print len(current_transf)

            #hold = True if current_holding[1] == 1 else False
            #print hold
            #print current_stud
            sys.stdin.readline()
            z_axis, path_constr, goal_constr = run_PCA(current_points, current_transf, current_holding, stud_offset = [0], plot = False, draw_axes = True, verbose = True)

            #print goal_constr[0]
            #print goal_constr[1]
            #print goal_constr[2]
            print"boundary matrix is: \n"
            print path_constr[2]
            print "draw ee_transform"
            #drawingHandles.append(misc.DrawAxes(env, goal_constr[0], 1))
            #drawingHandles.append(misc.DrawAxes(env, current_transf[-1], 1))
            raw_input("Press enter for the next segment...")

            list_of_actions.append(["R", [5, 7], True, path_constr[0], path_constr[1], path_constr[2],
                                   goal_constr[0], goal_constr[1], goal_constr[2]])

        pickle.dump(list_of_actions, open(TSR_filename, "wb"))

    raw_input("Press enter to exit...")

