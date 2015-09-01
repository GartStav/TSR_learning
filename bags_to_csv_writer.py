#!/usr/bin/env python

'''
bags_to_csv_writer.py
Write transforms from a rosbag into csv files. Rosbag should be playing.

Author: Artem Gritsenko
Worcester Polytechnic Institute, ArcLab
July 2015
'''

import rospy
import tf
import csv

if __name__ == '__main__':
    rospy.init_node('tf_saver')

    bag_name  = "UnhangHangWithoutHandArtem3"

    filename_hand_r = "bags_csv/" + bag_name + "/hand_r_" + bag_name + ".csv"
    filename_hand_l = "bags_csv/" + bag_name + "/hand_l_" + bag_name + ".csv"
    filename_obj = "bags_csv/" + bag_name + "/wheel_" + bag_name + ".csv"

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    # clean files
    open(filename_hand_r, 'w').close()
    open(filename_hand_l, 'w').close()
    open(filename_obj, 'w').close()

    while not rospy.is_shutdown():

        try:
            (trans, rot) = listener.lookupTransform('/mocap_world', '/mocap/RHAND/RHAND', rospy.Time(0))
            (trans2, rot2) = listener.lookupTransform('/mocap_world', '/mocap/LHAND/LHAND', rospy.Time(0))
            (trans3, rot3) = listener.lookupTransform('/hub_LF', '/wheel_LF', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        with open(filename_hand_r, 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            #print trans + rot
            writer.writerow(trans + rot)

        with open(filename_hand_l, 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            #print trans2 + rot2
            writer.writerow(trans2 + rot2)

        with open(filename_obj, 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            #print trans3 + rot3
            writer.writerow(trans3 + rot3)

        rate.sleep()
