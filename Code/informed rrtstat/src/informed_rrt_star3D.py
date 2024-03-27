# informed RRT star in 3D

import numpy as np
import matplotlib.pyplot as plt
import time
import copy
import importlib

import os
import sys


# from env3D import env
import env3D
import utils3D
import plot_util3D


importlib.reload(utils3D)
importlib.reload(env3D)
importlib.reload(plot_util3D)

def CreateUnitSphere(r = 1):
    phi = np.linspace(0,2*np.pi, 256).reshape(256, 1) # the angle of the projection in the xy-plane
    theta = np.linspace(0, np.pi, 256).reshape(-1, 256) # the angle from the polar axis, ie the polar angle
    radius = r

    # Transformation formulae for a spherical coordinate system.
    x = radius*np.sin(theta)*np.cos(phi)
    y = radius*np.sin(theta)*np.sin(phi)
    z = radius*np.cos(theta)
    return (x, y, z)

# def draw_ellipsoid(ax, C, L, xcenter):
#     (xs, ys, zs) = CreateUnitSphere()
#     pts = np.array([xs, ys, zs])
#     pts_in_world_frame = C@L@pts + xcenter
#     ax.plot_surface(pts_in_world_frame[0], pts_in_world_frame[1], pts_in_world_frame[2], alpha=0.05, color="g")

class IRRT:

    def __init__(self,  show_ellipse = False):
        self.env = env3D.env()
        self.xstart, self.xgoal = tuple(self.env.start), tuple(self.env.goal)
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.Parent = {}
        self.Path = []
        self.N = 4000 # used for determining how many batches needed
        self.ind = 0
        self.i = 0
        # rrt* near and other utils
        self.stepsize = 1
        self.gamma = 500
        self.eta = self.stepsize
        self.rgoal = self.stepsize
        self.done = False
        # for drawing the ellipse
        self.C = np.zeros([3,3])
        self.L = np.zeros([3,3])
        self.xcenter = np.zeros(3)
        self.show_ellipse = show_ellipse

    def Informed_rrt(self):
        self.V = [self.xstart]
        self.E = set()
        self.Xsoln = set()
        self.T = (self.V, self.E)
        
        c = 1
        while self.ind <= self.N:
            print(self.ind)
            plot_util3D.visualization(self)
            # print(self.i)
            if len(self.Xsoln) == 0:
                cbest = np.inf
            else:
                cbest = min({self.cost(xsln) for xsln in self.Xsoln})
            xrand = self.Sample(self.xstart, self.xgoal, cbest)
            xnearest = utils3D.nearest(self, xrand)
            xnew, dist = utils3D.steer(self, xnearest, xrand)
            # print(xnew)
            collide, _ = utils3D.isCollide(self, xnearest, xnew, dist=dist)
            if not collide:
                self.V.append(xnew)
                Xnear = utils3D.near(self, xnew)
                xmin = xnearest
                cmin = self.cost(xmin) + c * self.line(xnearest, xnew)
                for xnear in Xnear:
                    xnear = tuple(xnear)
                    cnew = self.cost(xnear) + c * self.line(xnear, xnew)
                    if cnew < cmin:
                        collide, _ = utils3D.isCollide(self, xnear, xnew)
                        if not collide:
                            xmin = xnear
                            cmin = cnew
                self.E.add((xmin, xnew))
                self.Parent[xnew] = xmin
                
                for xnear in Xnear:
                    xnear = tuple(xnear)
                    cnear = self.cost(xnear)
                    cnew = self.cost(xnew) + c * self.line(xnew, xnear)
                    # rewire
                    if cnew < cnear:
                        collide, _ = utils3D.isCollide(self, xnew, xnear)
                        if not collide:
                            xparent = self.Parent[xnear]
                            self.E.difference_update((xparent, xnear))
                            self.E.add((xnew, xnear))
                            self.Parent[xnear] = xnew
                self.i += 1
                if self.InGoalRegion(xnew):
                    print('reached')
                    self.done = True
                    self.Parent[self.xgoal] = xnew
                    self.Path, _ = utils3D.path(self)
                    self.Xsoln.add(xnew)
            # update path
            if self.done:
                self.Path,diss= utils3D.path(self, Path = [])
                # print(disss)
            self.ind += 1
        plot_util3D.visualization(self)
        # return tree
        return self.Path
                
    def Sample(self, xstart, xgoal, cmax, bias = 0.05):
        # sample within a eclipse 
        if cmax < np.inf:
            cmin = utils3D.getDist(xgoal, xstart)
            xcenter = np.array([(xgoal[0] + xstart[0]) / 2, (xgoal[1] + xstart[1]) / 2, (xgoal[2] + xstart[2]) / 2])
            C = self.RotationToWorldFrame(xstart, xgoal)
            r = np.zeros(3)
            r[0] = cmax /2
            for i in range(1,3):
                r[i] = np.sqrt(cmax**2 - cmin**2) / 2
            L = np.diag(r) # R3*3 
            xball = self.SampleUnitBall() # np.array
            x =  C@L@xball + xcenter
            self.C = C # save to global var
            self.xcenter = xcenter
            self.L = L
            if not utils3D.isinside(self, x): # intersection with the state space
                xrand = x
            else:
                return self.Sample(xstart, xgoal, cmax)
        else:
            xrand = utils3D.sampleFree(self, bias = bias)
        return xrand

    def SampleUnitBall(self):
        # uniform sampling in spherical coordinate system in 3D
        # sample radius
        r = np.random.uniform(0.0, 1.0)
        theta = np.random.uniform(0, np.pi)
        phi = np.random.uniform(0, 2 * np.pi)
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta)
        return np.array([x,y,z])

    def RotationToWorldFrame(self, xstart, xgoal):
        # S0(n): such that the xstart and xgoal are the center points
        d = utils3D.getDist(xstart, xgoal)
        xstart, xgoal = np.array(xstart), np.array(xgoal)
        a1 = (xgoal - xstart) / d
        M = np.outer(a1,[1,0,0])
        U, S, V = np.linalg.svd(M)
        C = U@np.diag([1, 1, np.linalg.det(U)*np.linalg.det(V)])@V.T
        return C

    def InGoalRegion(self, x):
        # Xgoal = {x in Xfree | \\x-xgoal\\2 <= rgoal}
        return utils3D.getDist(x, self.xgoal) <= self.rgoal

    def cost(self, x):
        # actual cost 
        '''here use the additive recursive cost function'''
        if x == self.xstart:
            return 0.0
        if x not in self.Parent:
            return np.inf
        return self.cost(self.Parent[x]) + utils3D.getDist(x, self.Parent[x])

    def line(self, x, y):
        return utils3D.getDist(x, y)

    # def visualization(self):
    #     if self.ind % 500 == 0:
    #         V = np.array(self.V)
    #         edges = list(map(list, self.E))
    #         Path = np.array(self.Path)
    #         start = self.env.start
    #         goal = self.env.goal
    #         # edges = E.get_edge()
    #         #----------- list structure
    #         # edges = []
    #         # for i in self.Parent:
    #         #     edges.append([i,self.Parent[i]])
    #         #----------- end
    #         # generate axis objects
    #         ax = plt.subplot(111, projection='3d')
            
    #         # ax.view_init(elev=0.+ 0.03*self.ind/(2*np.pi), azim=90 + 0.03*self.ind/(2*np.pi))
    #         # ax.view_init(elev=0., azim=90.)
    #         ax.view_init(elev=90., azim=0.)
    #         # ax.view_init(elev=-8., azim=180)
    #         ax.clear()
    #         # drawing objects
    #         #draw_Spheres(ax, self.env.balls)
    #         draw_block_list(ax, self.env.blocks)
    #         #if self.env.OBB is not None:
    #         #    draw_obb(ax, self.env.OBB)
    #         draw_block_list(ax, np.array([self.env.boundary]), alpha=0)
    #         draw_line(ax, edges, visibility=0.75, color='g')
    #         draw_line(ax, Path, color='r')
    #         if self.show_ellipse:
    #             draw_ellipsoid(ax, self.C, self.L, self.xcenter) # beware, depending on start and goal position, this might be bad for vis
    #         if len(V) > 0:
    #             ax.scatter3D(V[:, 0], V[:, 1], V[:, 2], s=2, color='g', )
    #         ax.plot(start[0:1], start[1:2], start[2:], 'go', markersize=7, markeredgecolor='k')
    #         ax.plot(goal[0:1], goal[1:2], goal[2:], 'ro', markersize=7, markeredgecolor='k')
    #         # adjust the aspect ratio
    #         ax.dist = 10
    #         set_axes_equal(ax)
    #         make_transparent(ax)
    #         #plt.xlabel('s')
    #         #plt.ylabel('y')
    #         # ax.set_axis_off()
    #         plt.pause(0.0001)

if __name__ == '__main__':
    A = IRRT(show_ellipse=False)
    A.Informed_rrt()
