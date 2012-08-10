#!/usr/bin/env python
# coding: latin-1
import roslib; roslib.load_manifest('formation_assignment')
import rospy
import math
import cmath
import TheHive

#Get instance of the hive for getting all robot positions
H = TheHive.get_hive_instance()


################################################################################
###
###  Calculates translation vector τ and rotation angle θ
###  in order to minimize the below cost function:
###
###  (τ[k], θ[k]) = arg min[τ,θ] L(Y[k], τ, θ)
###
###  where Y[k] is the cost function for a given assignment set.
###
################################################################################


def find_minimum_cost():
    """Minimizes the cost function for translation vector and rotation angle."""
    
    N = 4
    
    calculate_optimal_rotation(N)
    
    calculate_optimal_translation(N)
    
    
    
def calculate_optimal_rotation(N):
    """Calculates the optimal rotation angle θ for synthesizing formation.
    
    Optimal rotation angle is given by:
        θ = arctan(W1 / W2)         W1 = Σ[N, i=1] (xi - μx)^T(yi - μy)
                                    W2 = Σ[N, i=1] (xi - μx)^T (0 -1)(yi - μy)
                                                               (1  0)
                                    where μx and μy are the centroids of sets
                                    X and Y, respectively.
                                    X = {x1,x2,...,xN} ∈ R2     current position set
                                    Y = {y1,y2,...,yN} ∈ R2     desired position set
                                    
    """
    
    centroidSetX = find_centroid(N)
    centroidSetY = find_centroid(N)
    
    print centroidSetX



def calculate_optimal_translation(N):
    """Calculates the optimal translation vector τ for synthesizing formation.
    
    Optimal translation vector is given by:
        τ = 1/N Σ[N, i=1] xi  -  1/N Σ[N, i=1] yi
        
    """
    
    pass
    
    
    
    
def find_centroid(N):
    """Finds the centroid of a set of points in 2D space.
    
    For a finite set of k points X = x1, x2, ... , xk ∈ Rn,
    the centroid of the set is C = (x1 + x2 + ... + xk) / k.
    
    In this case, k is the number of robots in the formation (k = N).
    
    """
    
    p1, p2, p3, p4 = H.position1, H.position2, H.position3, H.position4
    
    xCoord = p1[0] + p2[0] + p3[0] + p4[0]
    
    yCoord = p1[1] + p2[1] + p3[1] + p4[1]
    
    centroid = [xCoord/N, yCoord/N]
        
    return centroid
    
    
    
if __name__ == "__main__":
    find_minimum_cost()
        
    