
import numpy as np
# from numpy.matlib import repmat
from collections import defaultdict
import time

import os
import sys
import importlib

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Sampling_based_Planning/")
import env3D
# import plot_util3D
import utils3D
from utils3D import getDist, nearest, steer, isCollide, near, cost, path

importlib.reload(utils3D)
importlib.reload(env3D)
# importlib.reload(plot_util3D)

class rrtstar():
    def __init__(self):
        self.env = env3D.env()

        self.Parent = {}
        self.V = []
        self.COST = {}

        self.i = 0
        self.maxiter = 4000 
        self.stepsize = 2
        self.gamma = 7
        self.eta = self.stepsize
        self.Path = []
        self.done = False
        self.intial = tuple(self.env.start)
        self.final = tuple(self.env.goal)

        self.V.append(self.intial)
        self.ind = 0
    def wireup(self,x,y):
        self.Parent[x] = y

    def removewire(self,xnear):
        xparent = self.Parent[xnear]
        a = [xnear,xparent]

    def reached(self):
        self.done = True
        goal = self.final
        xn = utils3D.near(self,self.env.goal)
        c = [utils3D.cost(self,tuple(x)) for x in xn]
        xncmin = xn[np.argmin(c)]
        self.wireup(goal , tuple(xncmin))
        self.V.append(goal)
        self.Path,self.D = utils3D.path(self)

    def run(self):
        xnew = self.intial
        print('starting rrt*... ')
        starttime = time.time()
        # self.fig = plt.figure(figsize = (10,8))
        while self.ind < self.maxiter:
            print(self.ind)
            xrand    = utils3D.Randomsample(self)
            xnearest = utils3D.nearest(self,xrand)
            xnew, dist  = utils3D.steer(self,xnearest,xrand)
            collide, _ = utils3D.isCollide(self,xnearest,xnew,dist=dist)
            if not collide:
                Xnear = utils3D.near(self,xnew)
                self.V.append(xnew) # add point
                # plot_util3D.visualization(self)
                # plt.title('rrt*')
                # minimal path and minimal cost
                xmin, cmin = xnearest, utils3D.cost(self, xnearest) + utils3D.getDist(xnearest, xnew)
                # connecting along minimal cost path
                Collide = []
                for xnear in Xnear:
                    xnear = tuple(xnear)
                    c1 = utils3D.cost(self, xnear) + utils3D.getDist(xnew, xnear)
                    collide, _ = utils3D.isCollide(self, xnew, xnear)
                    Collide.append(collide)
                    if not collide and c1 < cmin:
                        xmin, cmin = xnear, c1
                self.wireup(xnew, xmin)
                # rewire
                for i in range(len(Xnear)):
                    collide = Collide[i]
                    xnear = tuple(Xnear[i])
                    c2 = utils3D.cost(self, xnew) + utils3D.getDist(xnew, xnear)
                    if not collide and c2 < utils3D.cost(self, xnear):
                        # self.removewire(xnear)
                        self.wireup(xnear, xnew)
                self.i += 1
            self.ind += 1
        # max sample reached
        self.reached()
        print('time used = ' + str(time.time()-starttime))
        print('Total distance = '+str(self.D))
        # plot_util3D.visualization(self)
        return self.Path
        
if __name__ == '__main__':
    p = rrtstar()
    p.run()
    
