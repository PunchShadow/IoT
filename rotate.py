# Input a 1x3 numpy vector and get its orthogonal vector
import numpy as np
import math

def RotateMap(vector, direction):
    rot_clo = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    rot_count = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    
    # direction = 1: clockwise
    if direction is 1:
        return np.dot(rot_clo, vector)
    else:
        return np.dot(rot_count, vector)

# Find a cross b's direction (1x3)
# Return 1 if Z is positive, else -1.
def CrossDirection(a, b):
    result = np.cross(a, b)
    if result[2] < 0:
        return -1;
    else:
        return 1;


# Project vector a to direction
# Output (projection, rejection)
def VectorProjection(a, direction):
    cos = np.dot(direction, a) / (np.linalg.norm(direction) * np.linalg.norm(a))
    projection = (np.dot(direction, a) / np.dot(a, a)) * direction
    rejection = a - projection
    return projection, rejection

# If two vector's multiplication's elements are all positive,
# two vectors are same direction
def SameDirection(a, project_a):
    if ((np.sign(a) * np.sign(project_a)).all() >= 0): 
        return 1
    else:
        return 0


# Determine if a,b is the same direction by minus them
# If a, b is the same direction, return 1, else 0
def FindSameDirection(a, b):
    # If either a or b is zero vector, return -1
    if (np.linalg.norm(a) is 0) or (np.linalg.norm(b) is 0):
        return -1;
    else:
        norm_a = a / np.linalg.norm(a)
        norm_b = b / np.linalg.norm(b)
        c = norm_a - norm_b
        # a, b is the same direction
        if (np.linalg.norm(c) < math.sqrt(2)):
            return 1
        else:
            return 0

arr = np.array([8,-2,0])
arr2 = np.array([5, -10, 0])
#print(RotateMap(arr, 1))
#print(CrossDirection(arr, arr2))
#print(FindSameDirection(arr,arr2))

pro, rej = VectorProjection(arr, arr2)
print(pro, rej)
print(SameDirection(arr, pro))
