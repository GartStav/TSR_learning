if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

import numpy as np
import csv

from wpi_planning_utilities.transformation_helper import *


class TSR_extractor():

    def __init__(self):

        print "Setting up the environment..."
        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.Reset()
        self.env.Load('robots/pr2-beta-static.zae')
        self.robot = self.env.GetRobots()[0]
        self.robot.SetActiveManipulator("rightarm")
        self.time.sleep(0.1)
        self.drawingHandles = []

    def remove_duplicates(self, points, trans, holding):

        new_points = []
        new_trans = []
        new_holding = []

        if len(points) > 0:
            new_points.append(points[0])
            new_trans.append(trans[0])
            new_holding.append(holding[0])
            for i in range(1, len(points)):
                if not (points[i] == points[i-1]).all():
                    new_points.append(points[i])
                    new_trans.append(trans[i])
                    new_holding.append(holding[i])

        return new_points, new_trans, new_holding


    def get_ee_poses_from_bag(self, filename):

        ee_points = np.zeros((0, 3))
        ee_trans = []
        holding = []
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for row in reader:
                ee_points = np.append(ee_points, [asarray([float(row[0]), float(row[1]), float(row[2])])], axis = 0)
                new_trans = BuildMatrix([float(row[i]) for i in range(3)], [float(row[i]) for i in range(3, 6)])
                ee_trans.append(new_trans)
                holding.append(float(row[6]))

        print len(ee_points)

        # SEGMENT OUT UNSCREW FROM SCREW - BAG 2 [0:48], [48:]
        ee_points, ee_trans = self.remove_duplicates(ee_points[48:], ee_trans[48:])

        print len(ee_points)

        colors = []
        [colors.append((0, 0, 1)) for x in xrange(0, len(ee_points))]

        self.drawingHandles.append(self.env.plot3( points = asarray(ee_points), pointsize=3.0, colors = asarray(colors) ))

        raw_input("Press ENTER. Points are drawn..")

        #draw all points one after another
        if False:
            del self.drawingHandles[:]
            colors = []
            colors.append((0.6, 0.6, 0.0))
            for i in range(len(ee_points)):
                self.drawingHandles.append(self.env.plot3( points = asarray(ee_points[i]), pointsize=3.0, colors = asarray(colors) ))
                print "point %r is drawn" %i
                sys.stdin.readline()

        #[20, 53, 73, 82, 93, len(dot_products)] # 5 segment is constrained, unscrew segment not detected #114
        #[20, 37, 67, 85, 96, len(dot_products)] # 2nd segment and 5 segment are constrained #115
        #[20, 34, 66, 77, 83, len(dot_products)] # 2nd and 5 segments are constrained #97
        #[21, 36, 76, 87, 94, len(dot_products)] # 2nd and 5 segments are constrained #116

        # draw segments with different colors
        if False:

            colors1 = []
            [colors1.append((1, 0, 0)) for x in xrange(0, len(ee_points))]

            self.drawingHandles.append(self.env.plot3( points = asarray(ee_points[0:21]), pointsize=3.0, colors = asarray(colors1[0:21]) ))

            raw_input("Press ENTER. first segment is drawn..")

            colors2 = []
            [colors2.append((0, 0, 1)) for x in xrange(0, len(ee_points))]

            self.drawingHandles.append(self.env.plot3( points = asarray(ee_points[21:38]), pointsize=3.0, colors = asarray(colors2[21:38]) ))

            raw_input("Press ENTER. second segment is drawn..")

            colors3 = []
            [colors3.append((0, 0.4, 0)) for x in xrange(0, len(ee_points))]

            self.drawingHandles.append(self.env.plot3( points = asarray(ee_points[36:48]), pointsize=3.0, colors = asarray(colors3[36:48]) ))

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

        return ee_points, ee_trans, holding

    def plot_smth(self):
        print "plot"

    # run PCA
    def PCA(self):
        print "PCA"

    # find changepoints
    def find_changepoints(self):
        self.PCA()


    # extract TSRs from the segment of primitive motion, i.e. reaching part of unscrew motion
    def extract_TSRs_from_segment(self, segment_points, ee_trans, holding, plot = False, draw_axes = False, verbose = False, eigenvalue_threshold = 0.0001):


        self.PCA()

        mean_x = np.mean(segment_points[:, 0])
        mean_y = np.mean(segment_points[:, 1])
        mean_z = np.mean(segment_points[:, 2])

        mean_vector = np.array([[mean_x], [mean_y], [mean_z]])

        if verbose: print 'Mean Vector:\n', mean_vector

        dim = 3

        scatter_matrix = np.zeros((dim, dim))

        for i in range(segment_points.shape[0]):
            scatter_matrix += (segment_points[i, :].reshape(dim, 1)
                - mean_vector).dot((segment_points[i, :].reshape(dim, 1) - mean_vector).T)
        if verbose: print 'Scatter Matrix:\n', scatter_matrix

        cov_mat = np.cov([segment_points[:, 0], segment_points[:, 1], segment_points[:, 2]])
        if verbose: print 'Covariance Matrix:\n', cov_mat

        # eigenvectors and eigenvalues for the from the scatter matrix
        eig_val_sc, eig_vec_sc = np.linalg.eig(scatter_matrix)

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

            plt.plot(segment_points[:, 0], segment_points[:, 1],
                segment_points[:, 2],
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
                a = Arrow3D([segment_points[0, 0], v[0]+segment_points[0, 0]], [segment_points[0, 1], v[1]+segment_points[0, 1]],
                    [segment_points[0, 2], v[2]+segment_points[0, 2]],
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
        for ev in eig_vec_sc:
            np.testing.assert_array_almost_equal(1.0, np.linalg.norm(ev))

        # Make a list of (eigenvalue, eigenvector) tuples
        eig_pairs = [(np.abs(eig_val_sc[i]), eig_vec_sc[:, i]) for i in range(len(eig_val_sc))]

        # Sort the (eigenvalue, eigenvector) tuples from high to low
        eig_pairs.sort()
        eig_pairs.reverse()

        matrix_w = np.hstack((eig_pairs[0][1].reshape(3, 1), eig_pairs[1][1].reshape(3, 1), eig_pairs[2][1].reshape(3, 1)))
        if verbose: print 'New coordinates:\n', matrix_w

        transformed = matrix_w.T.dot(segment_points.T)

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
        TSR_0_w_path = MakeTransform(matrix_w, matrix(segment_points[0, :]))
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
        #print segment_points[0, :]
        #print transformed[:, 0]
        #print B_w_path
        #sys.stdin.readline()

        # Goal TSR frame
        TSR_0_w_goal = ee_trans[-1]
        # Goal EE offset
        EE_offset_goal = MakeTransform(eye(3), matrix([0, 0, 0]))
        # Goal B_w
        B_w_goal = mat([-0, 0, -0, 0, -0, 0, -0, 0, -0, 0, -0, 0])


        if draw_axes:
            #draw TSR_frame
            drawingHandles.append(misc.DrawAxes(env, TSR_0_w_path, 1))
            drawingHandles.append(misc.DrawAxes(env, TSR_0_w_goal, 1))
        new_z_axis = eig_pairs[0][1]

        plt.show(block = False)

        return new_z_axis, [TSR_0_w_path, EE_offset_path, B_w_path], [TSR_0_w_goal, EE_offset_goal, B_w_goal]
