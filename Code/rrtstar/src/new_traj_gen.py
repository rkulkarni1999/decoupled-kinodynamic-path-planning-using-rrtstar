import numpy as np
from collections import namedtuple
import numpy as np
from numpy import linalg as LA
from scipy import optimize
# from .trajutils import *

DesiredState = namedtuple('DesiredState', 'pos vel acc jerk yaw yawdot')

def polyder(t, k = 0, order = 10):
    if k == 'all':
        terms = np.array([polyder(t,k,order) for k in range(1,5)])
    else:
        terms = np.zeros(order)
        coeffs = np.polyder([1]*order,k)[::-1]
        pows = t**np.arange(0,order-k,1)
        terms[k:] = coeffs*pows
    return terms

def Hessian(T,order = 10,opt = 4):
    n = len(T)
    Q = np.zeros((order*n,order*n))
    for k in range(n):
        m = np.arange(0,opt,1)
        for i in range(order):
            for j in range(order):
                if i >= opt and j >= opt:
                    pow = i+j-2*opt+1
                    Q[order*k+i,order*k+j] = 2*np.prod((i-m)*(j-m))*T[k]**pow/pow
    return Q

def Circle_waypoints(n,Tmax = 2*np.pi):
    t = np.linspace(0,Tmax, n)
    x = 1+0.5*np.cos(t)
    y = 1+0.5*np.sin(t)
    z = 1+0*t
    return np.stack((x, y, z), axis=-1)

def Helix_waypoints(n,Tmax = 2*np.pi):

    t = np.linspace(0, Tmax, n)
    x = 1+0.5*np.cos(t)
    y = 1+0.5*np.sin(t)
    z = t/Tmax*2

    return np.stack((x, y, z), axis=-1)

class trajGenerator:
    def __init__(self,waypoints,max_vel = 0.3,gamma = 100):
        self.waypoints = waypoints
        self.max_vel = max_vel
        self.gamma = gamma
        self.order = 10
        len,dim = waypoints.shape
        self.dim = dim
        self.len = len
        self.TS = np.zeros(self.len)
        self.optimize()
        self.yaw = 0
        self.heading = np.zeros(2)

    def get_cost(self,T):
        coeffs,cost = self.MinimizeSnap(T)
        cost = cost + self.gamma*np.sum(T)
        return cost

    def optimize(self):
        diff = self.waypoints[0:-1] - self.waypoints[1:]
        Tmin = LA.norm(diff,axis = -1)/self.max_vel
        T = optimize.minimize(self.get_cost,Tmin, method="COBYLA",constraints= ({'type': 'ineq', 'fun': lambda T: T-Tmin}))['x']

        self.TS[1:] = np.cumsum(T)
        self.coeffs, self.cost = self.MinimizeSnap(T)


    def MinimizeSnap(self,T):
        unkns = 4*(self.len - 2)

        Q = Hessian(T)
        A,B = self.get_constraints(T)

        invA = LA.inv(A)

        if unkns != 0:
            R = invA.T@Q@invA

            Rfp = R[:-unkns,-unkns:]
            Rpp = R[-unkns:,-unkns:]

            B[-unkns:,] = -LA.inv(Rpp)@Rfp.T@B[:-unkns,]

        P = invA@B
        cost = np.trace(P.T@Q@P)

        return P, cost

    def get_constraints(self,T):
        n = self.len - 1
        o = self.order

        A = np.zeros((self.order*n, self.order*n))
        B = np.zeros((self.order*n, self.dim))

        B[:n,:] = self.waypoints[ :-1, : ]
        B[n:2*n,:] = self.waypoints[1: , : ]

        #waypoints contraints
        for i in range(n):
            A[i, o*i : o*(i+1)] = polyder(0)
            A[i + n, o*i : o*(i+1)] = polyder(T[i])

        #continuity contraints
        for i in range(n-1):
            A[2*n + 4*i: 2*n + 4*(i+1), o*i : o*(i+1)] = -polyder(T[i],'all')
            A[2*n + 4*i: 2*n + 4*(i+1), o*(i+1) : o*(i+2)] = polyder(0,'all')

        #start and end at rest
        A[6*n - 4 : 6*n, : o] = polyder(0,'all')
        A[6*n : 6*n + 4, -o : ] = polyder(T[-1],'all')

        #free variables
        for i in range(1,n):
            A[6*n + 4*i : 6*n + 4*(i+1), o*i : o*(i+1)] = polyder(0,'all')

        return A,B

    def get_des_state(self,t):

        if t > self.TS[-1]: t = self.TS[-1] - 0.001

        i = np.where(t >= self.TS)[0][-1]

        t = t - self.TS[i]
        coeff = (self.coeffs.T)[:,self.order*i:self.order*(i+1)]

        pos  = coeff@polyder(t)
        vel  = coeff@polyder(t,1)
        accl = coeff@polyder(t,2)
        jerk = coeff@polyder(t,3)

        #set yaw in the direction of velocity
        yaw, yawdot = self.get_yaw(vel[:2])

        return DesiredState(pos, vel, accl, jerk, yaw, yawdot)

    def get_yaw(self,vel):
        curr_heading = vel/LA.norm(vel)
        prev_heading = self.heading
        cosine = max(-1,min(np.dot(prev_heading, curr_heading),1))
        dyaw = np.arccos(cosine)
        norm_v = np.cross(prev_heading,curr_heading)
        self.yaw += np.sign(norm_v)*dyaw

        if self.yaw > np.pi: self.yaw -= 2*np.pi
        if self.yaw < -np.pi: self.yaw += 2*np.pi

        self.heading = curr_heading
        yawdot = max(-30,min(dyaw/0.005,30))
        return self.yaw,yawdot

    # def collision_optimize(self,Map):
    #     check = True
    #     while not check:
    #         self.optimize()
    #         check = self.check_collision(Map)
    #
    # def check_collision(self,Map):
    #     for t in np.linspace(0,self.TS[-1],1000):
    #         pos = self.get_des_state(t).pos
    #         if Map.idx.count((*pos,)) != 0:
    #             i = np.where(t >= self.TS)[0][-1]
    #             new_waypoint = (waypoints[i,:]+waypoints[i+1,:])/2
    #             np.insert(self.waypoints,i,new_waypoint)
    #             return True
    #     return False

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# from TrajGen import trajGenerator


def new_traj_gen(waypoints):
# Redefine the waypoints from the provided code
# waypoints = np.array([(5.0, 17.0, 2.0),
#  (4.707526804631064, 16.902793925191784, 2.8502401255179888),
#  (4.849188783948092, 16.741662013709877, 3.222247549957932),
#  (4.951386170891119, 16.04073905906404, 3.032313835842028),
#  (5.445143902341972, 14.892361896641646, 3.1526991196254563),
#  (6.164776064462577, 14.350182592362675, 3.3365004389977466),
#  (6.254596793191187, 12.36063841706286, 3.5199294749251027),
#  (6.344417521919796, 10.371094241763045, 3.7033585108524587),
#  (5.788618796827702, 8.449874254115448, 3.7045979661257467),
#  (5.744481808865578, 6.455781218608851, 3.8517199972350165),
#  (5.6610733100610435, 4.822224274287095, 3.4096983352745753),
#  (5.154237321479032, 3.6522850922512546, 3.4237601728006934),
#  (5.4276306366022276, 1.9810337838489636, 2.4982933677131633),
#  (5.113444716269026, 0.5772268419826383, 1.135188248135702),
#  (5.151831117951446, -0.6634959328182237, 0.9301579099364596),
#  (5.431446820512459, -1.69899511954097, 0.11304481629071139),
#  (5.0, -3.0, 0.0)])

    # Reverse and round the waypoints
    waypoints = waypoints[::-1]
    waypoints = np.round(waypoints, 2)

    # Initialize the trajectory generator
    # Note: The trajGenerator class and related functions must be already defined in the script
    traj_gen = trajGenerator(waypoints)

    # Define the time range for the trajectory
    total_time = traj_gen.TS[-1]
    time_step = 0.01                                           # Define a suitable time step
    time_traj = np.arange(0, total_time, time_step)

    # Initialize lists to store trajectory data
    px, py, pz = [], [], []
    vx, vy, vz = [], [], []
    ax, ay, az = [], [], []
    jerk_x, jerk_y, jerk_z = [], [], []
    yaw_traj, yaw_rate_traj = [], []

    # Extract trajectory information at each time step
    for t in time_traj:
        des_state = traj_gen.get_des_state(t)
        px.append(des_state.pos[0])
        py.append(des_state.pos[1])
        pz.append(des_state.pos[2])
        vx.append(des_state.vel[0])
        vy.append(des_state.vel[1])
        vz.append(des_state.vel[2])
        ax.append(des_state.acc[0])
        ay.append(des_state.acc[1])
        az.append(des_state.acc[2])
        jerk_x.append(des_state.jerk[0])
        jerk_y.append(des_state.jerk[1])
        jerk_z.append(des_state.jerk[2])
        yaw_traj.append(des_state.yaw)
        yaw_rate_traj.append(des_state.yawdot)

    # Convert lists to numpy arrays for easier handling
    px, py, pz = np.array(px), np.array(py), np.array(pz)
    vx, vy, vz = np.array(vx), np.array(vy), np.array(vz)
    ax, ay, az = np.array(ax), np.array(ay), np.array(az)
    jerk_x, jerk_y, jerk_z = np.array(jerk_x), np.array(jerk_y), np.array(jerk_z)
    yaw_traj, yaw_rate_traj = np.array(yaw_traj), np.array(yaw_rate_traj)

    MP_trajectory_matrix = np.array([ [px],[py], [pz],
                                     [vx],[vy],[vz],
                                      [ax],[ay],[az],[yaw_rate_traj] ])
    MP_trajectory_matrix = MP_trajectory_matrix.reshape(MP_trajectory_matrix.shape[0], MP_trajectory_matrix.shape[2])
    print(MP_trajectory_matrix)
    # Create the plots
    plt.figure(figsize=(15, 10))

    # 3D plot for position trajectory
    ax1 = plt.subplot(2, 3, 1, projection='3d')
    ax1.plot(px, py, pz, label='Position Trajectory')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_zlabel('Z Position')
    ax1.set_title('3D Position Trajectory')
    ax1.legend()

    # 2D plot for velocity trajectories
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(time_traj, vx, label='X Velocity')
    ax2.plot(time_traj, vy, label='Y Velocity')
    ax2.plot(time_traj, vz, label='Z Velocity')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity')
    ax2.set_title('Velocity Trajectories')
    ax2.legend()

    # 2D plot for acceleration trajectories
    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(time_traj, ax, label='X Acceleration')
    ax3.plot(time_traj, ay, label='Y Acceleration')
    ax3.plot(time_traj, az, label='Z Acceleration')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration')
    ax3.set_title('Acceleration Trajectories')
    ax3.legend()

    # 2D plot for jerk trajectories
    ax4 = plt.subplot(2, 3, 4)
    ax4.plot(time_traj, jerk_x, label='X Jerk')
    ax4.plot(time_traj, jerk_y, label='Y Jerk')
    ax4.plot(time_traj, jerk_z, label='Z Jerk')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Jerk')
    ax4.set_title('Jerk Trajectories')
    ax4.legend()

    # 2D plot for Yaw Angles
    ax5 = plt.subplot(2, 3, 5)
    ax5.plot(time_traj, yaw_traj, label='Yaw Angle', color='blue')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Yaw Angle (radians)')
    ax5.set_title('Yaw Angles Over Time')
    ax5.grid(True)
    ax5.legend()

    # 2D plot for Yaw Velocities
    ax6 = plt.subplot(2, 3, 6)
    ax6.plot(time_traj, yaw_rate_traj, label='Yaw Velocity', color='red')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Yaw Velocity (radians/s)')
    ax6.set_title('Yaw Velocities Over Time')
    ax6.grid(True)
    ax6.legend()

    plt.tight_layout()
    plt.show()
    return time_traj, MP_trajectory_matrix
