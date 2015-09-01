#!/usr/bin/env python
# Ben Suay, RAIL
# May 2013
# Worcester Polytechnic Institute
#

from openravepy import *
import sys
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
    import numpy
import time
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *
from math import *
from copy import *
import os # for file operations

class RaveCBiRRT:
    def __init__(self,env,kinbodyName):        
        self.env = env
        self.problem = RaveCreateModule(self.env,'CBiRRT')
        self.env.AddModule(self.problem,kinbodyName)
    
    def solve(self,myCommand):
        return self.problem.SendCommand(myCommand)
