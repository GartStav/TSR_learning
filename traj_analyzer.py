'''
!!!!!!!!!!!!!!!!!!!!!!

Author: Artem Gritsenko
Worcester Polytechnic Institute, ArcLab
July 2015
'''

import numpy as np

from itertools import tee, izip

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch

from wpi_planning_utilities.TransformMatrix import *
from wpi_planning_utilities.rodrigues import *

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

class TrajectoryAnalyzer():

    def __init__(self, env):
        print "init analyzer"
        self.env = env
        self.drawingHandles = []


    def sliding_window(self, iterable, size):
        iters = tee(iterable, size)
        for i in range(1, size):
            for each in iters[i:]:
                next(each, None)
        return izip(*iters)



    def find_segmentation_points(self, points, window_size = 10, to_plot = False):
        z_axis_vectors = []
        print "Compute segmentation for window size of " + str(window_size)
        print "Number of points is ", len(points)
        for each in self.sliding_window(points, window_size):
            window_points = asarray(each)
            eig_pairs = self.run_PCA(window_points)
            z_axis = eig_pairs[0][1]
            z_axis_vectors.append(z_axis)
        # dot products of a sliding window of consequtive z-vectors
        dot_products = np.zeros((len(z_axis_vectors), 7))
        for i in range(3, len(z_axis_vectors)-3):
            # for j in range(6):
            #     dot_products[i, j] = z_axis_windowed(i,:).dot(z_axis_windowed(i-j,:))
            #     dot(data(i,:), data(i-3,:))
            #     print i, j
            dot_products[i][0] = z_axis_vectors[i][:].dot(z_axis_vectors[i-3][:])
            dot_products[i][1] = z_axis_vectors[i][:].dot(z_axis_vectors[i-2][:])
            dot_products[i][2] = z_axis_vectors[i][:].dot(z_axis_vectors[i-1][:])
            dot_products[i][3] = z_axis_vectors[i][:].dot(z_axis_vectors[i+1][:])
            dot_products[i][4] = z_axis_vectors[i][:].dot(z_axis_vectors[i+2][:])
            dot_products[i][5] = z_axis_vectors[i][:].dot(z_axis_vectors[i+3][:])
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
            # if not extract_from_bags:
            #     for split in groundtruth_splits:
            #         plt.plot([split, split], [0, 1], '-', color='red', alpha=0.5)
            plt.xlabel('trajectory waypoints')
            plt.ylabel('z axis dot products')
            plt.title('Segmentation results for window of size ' + str(window_size))
            plt.show(block=False)


    def extr_TSR(self, window_points, ee_trans, hold, stud_offset=[], plot = False, draw_axes = False, verbose = False, eigenvalue_threshold = 0.0001):

            eig_pairs = self.run_PCA(window_points)

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

            # print np.min(transformed.T[:, 0])
            # print np.max(transformed.T[:, 0])
            # print np.min(transformed.T[:, 1])
            # print np.max(transformed.T[:, 1])
            # print np.min(transformed.T[:, 2])
            # print np.max(transformed.T[:, 2])

            # Path TSR frame
            TSR_0_w_path = MakeTransform(matrix_w, matrix(window_points[0, :]))
            change_axes = MakeTransform(rodrigues([0, pi/2, 0]), matrix([0, 0, 0]))*MakeTransform(rodrigues([0, 0, pi/2]), matrix([0, 0, 0]))
            TSR_0_w_path = TSR_0_w_path*change_axes
            # Path  EE offset
            EE_offset_path = MakeTransform(eye(3), matrix([0, 0, 0]))
            # Path B_w
            # print eig_pairs[0][0]
            # print eig_pairs[1][0]
            # print eig_pairs[2][0]
            # print eigenvalue_threshold
            # TODO fix the frames from extraction to execution
            z_min = np.min(transformed.T[:, 0])-transformed[0, 0]-0. if (eig_pairs[0][0] < eigenvalue_threshold) else -1000
            z_max = np.max(transformed.T[:, 0])-transformed[0, 0]+0. if (eig_pairs[0][0] < eigenvalue_threshold) else 1000
            x_min = np.min(transformed.T[:, 1])-transformed[1, 0]-0. if (eig_pairs[1][0] < eigenvalue_threshold) else -1000
            x_max = np.max(transformed.T[:, 1])-transformed[1, 0]+0. if (eig_pairs[1][0] < eigenvalue_threshold) else 1000
            y_min = np.min(transformed.T[:, 2])-transformed[2, 0]-0.2 if (eig_pairs[2][0] < eigenvalue_threshold) else -1000
            y_max = np.max(transformed.T[:, 2])-transformed[2, 0]+0.2 if (eig_pairs[2][0] < eigenvalue_threshold) else 1000
            B_w_path = mat([x_min, x_max, y_min, y_max, z_min, z_max, -1000, 1000, -1000, 1000, -1000, 1000])
            #print window_points[0, :]
            #print transformed[:, 0]
            #print B_w_path
            #sys.stdin.readline()

            # Goal TSR frame
            TSR_0_w_goal = ee_trans[-1]
            # Goal EE offset
            #EE_offset_goal = MakeTransform(eye(3), matrix([0, 0, 0]))
            EE_offset_goal = stud_offset[-1] # for the screw task the offset
            # Goal B_w
            B_w_goal = mat([-0, 0, -0, 0, -0, 0, -0, 0, -0, 0, -0, 0])


            if draw_axes:
                #draw TSR_frame
                self.drawingHandles.append(misc.DrawAxes(self.env, TSR_0_w_path, 0.3))
                self.drawingHandles.append(misc.DrawAxes(self.env, TSR_0_w_goal, 0.3))
            new_z_axis = eig_pairs[0][1]

            plt.show(block = False)

            return new_z_axis, [TSR_0_w_path, EE_offset_path, B_w_path], [TSR_0_w_goal, EE_offset_goal, B_w_goal]


    def run_PCA(self, points, plot = False, verbose = False):
        # calculate covariance matrix
        cov_mat = np.cov([points[:, 0], points[:, 1], points[:, 2]])
        if verbose: print 'Covariance Matrix:\n', cov_mat
        # eigenvectors and eigenvalues for the from the covariance matrix
        eig_val_cov, eig_vec_cov = np.linalg.eig(cov_mat)
        if verbose:
            for i in range(len(eig_val_cov)):
                print 'Eigenvector {}: \n{}'.format(i+1, eig_vec_cov[:, i])
                print 'Eigenvalue {} from covariance matrix: {}'.format(i+1, eig_val_cov[i])
                print 40 * '-'
        # check eigenvector-eigenvalue calculations
        # for i in range(len(eig_val_cov)):
        #     eigv = eig_vec_cov[:, i].reshape(1, 3).T
        #     np.testing.assert_array_almost_equal(cov_mat.dot(eigv),
        #             eig_vec_cov[i] * eigv, decimal=6,
        #             err_msg='', verbose=True)
        if plot:
            fig = plt.figure(1)
            ax = fig.add_subplot(111, projection='3d')
            plt.plot(points[:, 0], points[:, 1],
                points[:, 2],
                'o', markersize=8, color='green', alpha=0.2)
            plt.plot([np.mean(points[:, 0])], [np.mean(points[:, 1])],
                    [np.mean(points[:, 2])],
                'o', markersize=10, color='red', alpha=0.5)
            first = True
            for v in eig_vec_cov.T:
                c = 'b'
                if first:
                    c = 'r'
                    first = False
                a = Arrow3D([points[0, 0], v[0]+points[0, 0]], [points[0, 1], v[1]+points[0, 1]],
                    [points[0, 2], v[2]+points[0, 2]],
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
        eig_pairs = [(np.abs(eig_val_cov[i]), eig_vec_cov[:, i]) for i in range(len(eig_val_cov))]
        # Sort the (eigenvalue, eigenvector) tuples from high to low
        eig_pairs.sort()
        eig_pairs.reverse()
        matrix_w = np.hstack((eig_pairs[0][1].reshape(3, 1), eig_pairs[1][1].reshape(3, 1), eig_pairs[2][1].reshape(3, 1)))
        if verbose: print 'New coordinates:\n', matrix_w
        transformed = matrix_w.T.dot(points.T)
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
        return eig_pairs