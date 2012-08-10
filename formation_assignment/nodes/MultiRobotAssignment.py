#!/usr/bin/env python
# coding: latin-1
import roslib; roslib.load_manifest('formation_assignment')
import rospy
import math
import cmath


################################################################################
###
###  Calculates assignment set in order to minimize cost function Y:
###
###  Y[k] = arg min[Y] L(Y, τ[k-1], θ[k-1])
###
###  where τ is the translation vector and θ is the rotation angle.
###
################################################################################





