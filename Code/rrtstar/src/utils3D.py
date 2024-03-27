import numpy as np
from numpy.matlib import repmat
# import pyrr as pyrr
from collections import deque

import os
import sys




def getDist(pos1, pos2):
    return np.sqrt(sum([(pos1[0] - pos2[0]) ** 2, (pos1[1] - pos2[1]) ** 2, (pos1[2] - pos2[2]) ** 2]))


def Randomsample(initparams, bias = 0.5):
    '''biased sampling'''
    x = np.random.uniform(initparams.env.boundary[0:3], initparams.env.boundary[3:6])
    i = np.random.random()
    if isinside(initparams, x):
        return Randomsample(initparams)
    else:
        if i < bias:
            return np.array(initparams.final) + 1
        else:
            return x
        return x

# ---------------------- Collision checking algorithms
def isinside(initparams, x):
    '''see if inside obstacle'''
    for i in initparams.env.blocks:
        if isinbound(i, x):
            return True
    return False

def isinbound(i, x, mode = False, factor = 0.1, isarray = False):
    if isarray:
        compx = (i[0] - factor <= x[:,0]) & (x[:,0] < i[3] + factor) 
        compy = (i[1] - factor <= x[:,1]) & (x[:,1] < i[4] + factor) 
        compz = (i[2] - factor <= x[:,2]) & (x[:,2] < i[5] + factor) 
        return compx & compy & compz
    else:    
        return i[0] - factor <= x[0] < i[3] + factor and i[1] - factor <= x[1] < i[4] + factor and i[2] - factor <= x[2] < i[5]


def lineAABB(p0, p1, dist, aabb):

    mid = [(p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2]  # mid point
    I = [(p1[0] - p0[0]) / dist, (p1[1] - p0[1]) / dist, (p1[2] - p0[2]) / dist]  # unit direction
    hl = dist / 2  # radius
    T = [aabb.P[0] - mid[0], aabb.P[1] - mid[1], aabb.P[2] - mid[2]]
    # do any of the principal axis form a separting axis?
    if abs(T[0]) > (aabb.E[0] + hl * abs(I[0])): return False
    if abs(T[1]) > (aabb.E[1] + hl * abs(I[1])): return False
    if abs(T[2]) > (aabb.E[2] + hl * abs(I[2])): return False
    # I.cross(s axis) ?
    r = aabb.E[1] * abs(I[2]) + aabb.E[2] * abs(I[1])
    if abs(T[1] * I[2] - T[2] * I[1]) > r: return False
    # I.cross(y axis) ?
    r = aabb.E[0] * abs(I[2]) + aabb.E[2] * abs(I[0])
    if abs(T[2] * I[0] - T[0] * I[2]) > r: return False
    # I.cross(z axis) ?
    r = aabb.E[0] * abs(I[1]) + aabb.E[1] * abs(I[0])
    if abs(T[0] * I[1] - T[1] * I[0]) > r: return False

    return True


def isCollide(initparams, x, child, dist=None):
    if dist==None:
        dist = getDist(x, child)
    # check in bound
    if not isinbound(initparams.env.boundary, child): 
        return True, dist
    # check collision in AABB
    for i in range(len(initparams.env.AABB)):
        if lineAABB(x, child, dist, initparams.env.AABB[i]): 
            return True, dist
    return False, dist

# ---------------------- leaf node extending algorithms
def nearest(initparams, x, isset=False):
    V = np.array(initparams.V)
    if initparams.i == 0:
        return initparams.V[0]
    xr = repmat(x, len(V), 1)
    dists = np.linalg.norm(xr - V, axis=1)
    return tuple(initparams.V[np.argmin(dists)])

def near(initparams, x):
    # s = np.array(s)
    V = np.array(initparams.V)
    if initparams.i == 0:
        return [initparams.V[0]]
    cardV = len(initparams.V)
    eta = initparams.eta
    gamma = initparams.gamma
    # min{γRRT∗ (log(card (V ))/ card (V ))1/d, η}
    r = min(gamma * ((np.log(cardV) / cardV) ** (1/3)), eta)
    if initparams.done: 
        r = 1
    xr = repmat(x, len(V), 1)
    inside = np.linalg.norm(xr - V, axis=1) < r
    nearpoints = V[inside]
    return np.array(nearpoints)

def steer(initparams, x, y):
    # steer from s to y
    if np.equal(x, y).all():
        return x, 0.0
    dist, step = getDist(y, x), initparams.stepsize
    step = min(dist, step)
    increment = ((y[0] - x[0]) / dist * step, (y[1] - x[1]) / dist * step, (y[2] - x[2]) / dist * step)
    xnew = (x[0] + increment[0], x[1] + increment[1], x[2] + increment[2])
    # direc = (y - s) / np.linalg.norm(y - s)
    # xnew = s + initparams.stepsize * direc
    return xnew, dist

def cost(initparams, x):
    if x == initparams.intial:
        return 0
    return cost(initparams, initparams.Parent[x]) + getDist(x, initparams.Parent[x])

# def cost_from_set(initparams, x):
#     '''here use a incremental cost set function'''
#     if x == initparams.intial:
#         return 0
#     return initparams.COST[initparams.Parent[x]] + getDist(x, initparams.Parent[x])

def path(initparams, Path=[], dist=0):
    x = initparams.final
    while x != initparams.intial:
        x2 = initparams.Parent[x]
        Path.append(np.array([x, x2]))
        dist += getDist(x, x2)
        x = x2
    return Path, dist







        
        


