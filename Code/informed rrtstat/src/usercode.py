import numpy as np
import cv2
import bpy
import os
import scipy
import matplotlib.pyplot as plt
import informed_rrt_star3D
import importlib
import generate_trajectory
import time
import traj_gen_with_yaw

importlib.reload(generate_trajectory)
importlib.reload(informed_rrt_star3D)
importlib.reload(traj_gen_with_yaw)

class state_machine:
    def __init__(self, start_pos):
        # User code executes every dt time in seconds
        self.dt = 0.050

        ### Motion Profile - sample code - EDIT HERE! ######################################################
        # For tuning the PID, utilize the following code. Once tuned, modify this section as you want! 
        # self.MP = np.genfromtxt('./src/sample_traj/trajectories1.csv', delimiter=',', skip_header=0)
        self.activeIndex = 0
        ####################################################################################################
        A = informed_rrt_star3D.IRRT(show_ellipse=False)
        self.endpoints =  A.Informed_rrt()
        time.sleep(2)

        length = len(self.endpoints)+1

        waypoints = np.zeros((length,3))
        count = length -1
        for i in self.endpoints:
            waypoints[count] =  np.array([i[0][0],i[0][1],i[0][2]])
            count = count - 1
        i = self.endpoints[-1]
        waypoints[0] = np.array([i[1][0],i[1][1],i[1][2]])

        print(waypoints)

        # self.timetrash , self.MP = generate_trajectory.generate_trajectory_multiple_waypoints(waypoints, time_per_segment=3, dt=0.05)
        # print(waypoints)
        self.timetrash , self.MP = traj_gen_with_yaw.generate_smooth_trajectory(waypoints)
        # print(self.MP)
        # print(self.MP.shape)
        # print(self.endpoints.shape())
        self.timestep_array = np.array([])

        # POSITION
        self.current_x_pos = np.array([])
        self.current_y_pos = np.array([])
        self.current_z_pos = np.array([])

        self.desired_x_pos = np.array([])
        self.desired_y_pos = np.array([])
        self.desired_z_pos = np.array([])

        # VELOCITY
        self.current_x_vel = np.array([])
        self.current_y_vel = np.array([])
        self.current_z_vel = np.array([])

        self.desired_x_vel = np.array([])
        self.desired_y_vel = np.array([])
        self.desired_z_vel = np.array([])

        # Logger # [, 17, 2.0] , , 0.2
        self.time_array = 0
        self.x_sp_array = start_pos[0]
        self.y_sp_array = -start_pos[1]
        self.z_sp_array = -start_pos[2]

        self.x_array = 0
        self.y_array = 0
        self.z_array = 0 

        self.vx_array = 0
        self.vy_array = 0
        self.vz_array = 0
        self.vx_sp_array = 0
        self.vy_sp_array = 0
        self.vz_sp_array = 0

    def step(self, time, currpos, currvel):
        """
        Input: time, current position in blender frame (x, y, z)
        Output: desired position, velocity, acceleration, yaw (radians) as np.array([x_component, y_component, z_component]) in blender frame
        
        SI unit unless specified otherwise

        Sample Input:
            time = 0.0;
            currpos = np.array([1.0, 0.0, 1.0])
            currvel = np.array([1.0, 0.0, 1.0])

            Sample Output:
            xyz_desired = np.array([0.0, 0.0, 0.0])
            vel_desired = np.array([0.0, 0.0, 0.0]) 
            acc_desired = np.array([0.0, 0.0, 0.0])
            yaw_setpoint = 0.0


        """

        # EDIT HERE ###########################################################################################
        print(f"Shape of the Trajectory Matrix : {self.MP.shape}")
        ####################
        # CURRENT POSITION
        ####################
        print(f"\n===============================================")
        print(f"\nCurrent Position : ")
        print(f"\nx : {currpos[0]}, y : {currpos[1]}, z : {currpos[2]}")
        print(f"\n===============================================")


        ##################################################################################
        ############################################
        # UPDATING CURRENT POSITIONS AND VELOCITIES
        ############################################
        self.timestep_array = np.append(self.timestep_array, time)

        # Appending Current Position of the Robot.
        self.current_x_pos = np.append(self.current_x_pos, currpos[0])
        self.current_y_pos = np.append(self.current_y_pos, currpos[1])
        self.current_z_pos = np.append(self.current_z_pos, currpos[2]) 

        # Appending Current Velocity of the Robot.
        self.current_x_vel = np.append(self.current_x_vel, currvel[0])
        self.current_y_vel = np.append(self.current_y_vel, currvel[1])
        self.current_z_vel = np.append(self.current_z_vel, currvel[2])
        ##################################################################################


        # FOLLOW GENERATED TRAJECTORY
        xyz_desired = self.MP[:3, self.activeIndex]
        vel_desired = self.MP[3:6, self.activeIndex]
        acc_desired = self.MP[6:, self.activeIndex]
        acc_desired = self.MP[6:9, self.activeIndex]
        yaw_setpoint = self.MP[9, self.activeIndex]
        # yaw_setpoint = 0
        ##################################################################################
        ############################################
        # UPDATING CURRENT POSITIONS AND VELOCITIES
        ############################################
        
        # Appending the Desired Position of the Robot.
        self.desired_x_pos = np.append(self.desired_x_pos, xyz_desired[0])
        self.desired_y_pos = np.append(self.desired_y_pos, xyz_desired[1])
        self.desired_z_pos = np.append(self.desired_z_pos, xyz_desired[2])    

        # Appending the Desired Velocity of the Robot.
        self.desired_x_vel = np.append(self.desired_x_vel, vel_desired[0])
        self.desired_y_vel = np.append(self.desired_y_vel, vel_desired[1])
        self.desired_z_vel = np.append(self.desired_z_vel, vel_desired[2])

        #################################################################################

        # PERFORM COLLISION CHECKING HERE
        # collision = coll_check(...)
        # if collision:
        #     print('Robot has collided')
        #     exit(0)

        if self.activeIndex<len(self.MP[1, :]):
            self.activeIndex = self.activeIndex+1
        
        #######################################################################################################

        # logger
        self.time_array = np.vstack((self.time_array, time))
        self.x_array = np.vstack((self.x_array, currpos[0]))
        self.y_array = np.vstack((self.y_array, currpos[1]))
        self.z_array = np.vstack((self.z_array, currpos[2]))
        self.x_sp_array = np.vstack((self.x_sp_array, xyz_desired[0]))
        self.y_sp_array = np.vstack((self.y_sp_array, xyz_desired[1]))
        self.z_sp_array = np.vstack((self.z_sp_array, xyz_desired[2]))

        self.vx_array = np.vstack((self.vx_array, currvel[0]))
        self.vy_array = np.vstack((self.vy_array, currvel[1]))
        self.vz_array = np.vstack((self.vz_array, currvel[2]))
        self.vx_sp_array = np.vstack((self.vx_sp_array, vel_desired[0]))
        self.vy_sp_array = np.vstack((self.vy_sp_array, vel_desired[1]))
        self.vz_sp_array = np.vstack((self.vz_sp_array, vel_desired[2]))

        return xyz_desired, vel_desired, acc_desired, yaw_setpoint

    def terminate(self):
        loggedDict = {'time': self.time_array,
                  'x': self.x_array,
                  'y': self.y_array,
                  'z': self.z_array,
                  'x_des': self.x_sp_array,
                  'y_des': self.y_sp_array,
                  'z_des': self.z_sp_array,
                  'vx': self.vx_array,
                  'vy': self.vy_array,
                  'vz': self.vz_array,
                  'vx_des': self.vx_sp_array,
                  'vy_des': self.vy_sp_array,
                  'vz_des': self.vz_sp_array,
                  }  
        scipy.io.savemat('./log/user_states.mat', loggedDict)

        #############################################################################################
        ######################################
        # PLOTTING POSITIONS IN 3D FOR TUNING
        ######################################
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # Current
        ax.plot(self.current_x_pos, self.current_y_pos, self.current_z_pos, label='Position from Environment')
        # Desired
        ax.plot(self.desired_x_pos, self.desired_y_pos, self.desired_z_pos, label='Position from Trajectory')

        # Add labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Actual v/s Desired Positions')

        # Show the plot with a legend
        plt.legend()
        plt.savefig('POSITIONS_3D.png')
        #############################################################################################
        #################################################
        # PLOTTING VELOCITY V/S TIME FOR EVERY DIRECTION
        #################################################
        # Create a figure with three subplots
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))

        #####################
        # Plot X-Velocities
        #####################
        ax1.plot(self.timestep_array, self.current_x_vel, label='X-Velocity from Actual')
        ax1.plot(self.timestep_array, self.desired_x_vel, label='X-Velocity from Trajectory')
        ax1.axvline(x=2, color='red', linestyle='--', label='Time = 2')  # Add vertical line at time = 2
        ax1.set_xlabel('time')
        ax1.set_ylabel('X-Velocity')
        ax1.set_title('Actual vs. Desired X-Velocity')
        ax1.legend()

        ####################
        # Plot Y-Velocities
        ####################
        ax2.plot(self.timestep_array, self.current_y_vel, label='Y-Velocity from Actual')
        ax2.plot(self.timestep_array, self.desired_y_vel, label='Y-Velocity from Trajectory')
        ax2.axvline(x=2, color='red', linestyle='--', label='Time = 2')  # Add vertical line at time = 2
        ax2.set_xlabel('time')
        ax2.set_ylabel('Y-Velocity')
        ax2.set_title('Actual vs. Desired Y-Velocity')
        ax2.legend()

        #####################
        # Plot Z-Velocities
        #####################
        ax3.plot(self.timestep_array, self.current_z_vel, label='Z-Velocity from Actual')
        ax3.plot(self.timestep_array, self.desired_z_vel, label='Z-Velocity from Trajectory')
        ax3.axvline(x=2, color='red', linestyle='--', label='Time = 2')  # Add vertical line at time = 2
        ax3.set_xlabel('time')
        ax3.set_ylabel('Z-Velocity')
        ax3.set_title('Actual vs. Desired Z-Velocity')
        ax3.legend()

        # Adjust spacing between subplots
        plt.tight_layout()

        # Show the plot
        plt.savefig("XYZ_VELOCITIES.png")
        #############################################################################################
        #############################################################################################
        #################################################
        # PLOTTING POSITIONS V/S TIME FOR EVERY DIRECTION
        #################################################
        # Create a figure with three subplots
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))

        #####################
        # Plot X-Positions
        #####################
        ax1.plot(self.timestep_array, self.current_x_pos, label='X-Position from Actual')
        ax1.plot(self.timestep_array, self.desired_x_pos, label='X-Position from Trajectory')
        ax1.set_xlabel('time')
        ax1.set_ylabel('X-Position')
        ax1.set_title('Actual vs. Desired X-Position')
        ax1.legend()

        ####################
        # Plot Y-Positions
        ####################
        ax2.plot(self.timestep_array, self.current_y_pos, label='Y-Position from Actual')
        ax2.plot(self.timestep_array, self.desired_y_pos, label='Y-Position from Trajectory')     
        ax2.set_xlabel('time')
        ax2.set_ylabel('Y-Position')
        ax2.set_title('Actual vs. Desired Y-Position')
        ax2.legend()

        #####################
        # Plot Z-Positions
        #####################
        ax3.plot(self.timestep_array, self.current_z_pos, label='Z-Position from Actual')
        ax3.plot(self.timestep_array, self.desired_z_pos, label='Z-Position from Trajectory')
        ax3.set_xlabel('time')
        ax3.set_ylabel('Z-Position')
        ax3.set_title('Actual vs. Desired Z-Position')
        ax3.legend()

        # Adjust spacing between subplots
        plt.tight_layout()

        # Show the plot
        plt.savefig("XYZ_POSITIONS.png")

        print('user state machine terminted')


    def fetchLatestImage(self):
        # Fetch image - renders the camera, saves the rendered image to a file and reads from it. 
        path_dir = bpy.data.scenes["Scene"].node_tree.nodes["File Output"].base_path

        # Render Drone Camera
        cam = bpy.data.objects['DownCam']    
        bpy.context.scene.camera = cam
        bpy.context.scene.render.filepath = os.path.join(path_dir, 'DownCam_latest.png')
        bpy.ops.render.render(write_still=True)

        return cv2.imread(bpy.context.scene.render.filepath)
    
