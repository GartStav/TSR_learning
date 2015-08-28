import pickle
from numpy import *

# first translation then rotation

list_of_actions = [
        # label, base offset
        "B", [5], matrix([[1,0,0,0],[0,0,1,-0.02],[0,-1,0,0], [0,0,0,1]]), mat([-0, 0, -0, 0, -0, 0, -0, 0, -0, 0, -0, 0]), 0.7,
        # label, manipulator index, gripper angles, grab/release
        "G", [5], [0.54, 0], 0,
        # label, manipulator index, end-effector offset transform, boundary matrix
        "R", [5], matrix([[1,0,0,0],[0,0,1,-0.02],[0,-1,0,0], [0,0,0,1]]), mat([-0, 0, -0, 0, -0, 0, -0, 0, -0, 0, -0, 0]),
        "G", [5], [0.24, 0], 1,
        "R", [5], matrix([[1,0,0,0],[0,0,1,-0.23],[0,-1,0,0], [0,0,0,1]]), mat([0, 0, 0, 0, -0, 0, 0, 0, 0, 0, 0, 0])]

pickle.dump(list_of_actions, open("Unscrew", "wb"))

print "Description for Unscrew created"


list_of_actions = [
        # label, base offset
        #
        "B", [5], matrix([[1,0,0,0.],[0,-1,0,-1.6],[0,0,-1, 1.4], [0,0,0,1]]), mat([-0, 0, -0, 0, -0, 0, -0, 0, -0, 0, -0, 0]), 0.7,
        "R", [5], matrix([[1,0,0,0.],[0,-1,0,-1.6],[0,0,-1, 0.8], [0,0,0,1]]), mat([-0, 0, -0, 0, -0, 0, -0, 0, -0, 0, -0, 0]), #mat([-0.1, 0.1, -0.4, 0.4, 0, 0, 0, 0, -pi, pi, 0.0, 0.0]),
        # label, manipulator index, gripper angles, grab/release
        # "G", [5], [0.54, 0], 0]
        # label, manipulator index, end-effector offset transform, boundary matrix
        "R", [5], matrix([[1,0,0,0.],[0,-1,0,0],[0,0,-1,0.8], [0,0,0,1]]), mat([-0.1, 0.1, -0.4, 0.4, 0, 0, 0, 0, 0.0, 0.0, -2*pi, 2*pi]),
        "G", [5], [0.54, 0], 0,
        #"G", [5], [0.24, 0], 1,
        "R", [5], matrix([[1,0,0,0.7],[0,1,0,0.8],[0,0,1,1.2], [0,0,0,1]]), mat([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])]

pickle.dump(list_of_actions, open("Putdown", "wb"))

print "Description for Putdown created"


list_of_actions = [
        # label, base offset
        # TODO account for another grasp
        "B", [5], matrix([[0,0,-1, 0.15],[0,1,0,-0.22],[1,0,0,0.], [0,0,0,1]]), mat([0, 0, 0, 0, 0, 0, -pi, pi, 0, 0, 0, 0]), -2,
        "G", [5, 7], [0.34, 0.34], 0,
        "R", [5], matrix([[0,0,-1,0.13],[0,1,0,-0.22],[1,0,0,0], [0,0,0,1]]), mat([0, 0, 0, 0, 0, 0, -pi, pi, 0, 0, 0, 0]),
        "R", [7], matrix([[0,0,-1,0.13],[0,1,0,-0.22],[1,0,0,0], [0,0,0,1]]), mat([0, 0, 0, 0, 0, 0, -pi, pi, 0, 0, 0, 0]),
        "R", [5, 7], matrix([[0,0,-1,0.35],[0,1,0,0.],[1,0,0,0], [0,0,0,1]]), mat([0, 0, -1, 1, -1, 1,  0, 0, -pi, pi, 0, 0])]

        #"R", [5], matrix([[1,0,0,0.7],[0,1,0,0.8],[0,0,1,1.2], [0,0,0,1]]), mat([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])]

pickle.dump(list_of_actions, open("Unhang", "wb"))

print "Description for Unhang created"

