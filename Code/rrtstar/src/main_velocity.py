
import sys
import site
import csv
# PATH CONFIGURATION
user_site_packages =site.getusersitepackages()
sys.path.append("/home/asus/Hands_on_Aerial_Robotics/aerial/lib/python3.10/site-packages")
sys.path.append(user_site_packages) #For pip installed dependencies
sys.path.append('./src')

# IMPORT PIP LIBS
import importlib
import math
import os
import random
import numpy as np
import scipy
import rrt_star3D
import time
import generate_trajectory
import traj_gen_with_yaw
import new_traj_gen
    
import usercode
from djitellopy import Tello

importlib.reload(usercode)
import matplotlib.pyplot as plt

def get_trajectory(): # This function will remain the except for how the trajectory is generated. 
    # p = rrt_star3D.rrtstar()
    # endpoints = p.run()
    # time.sleep(2)

    # length = len(endpoints)+1

    # waypoints = np.zeros((length,3))
    # count = length -1
    # for i in endpoints:
    #     waypoints[count] =  np.array([i[0][0],i[0][1],i[0][2]])
    #     count = count - 1
    # i = endpoints[-1]
    # waypoints[0] = np.array([i[1][0],i[1][1],i[1][2]])

    # waypoints = np.array([  [ 0. ,         0. ,         1.        ],
    #                         [-0.81413407 , 1.1448905 ,  1.23811428],
    #                         [-0.87425354 , 2.59736626 , 1.60894123],
    #                         [ 0.08557153 , 3.29086545 , 1.65730888],
    #                         [ 0.5275638  , 3.26540295 , 1.72477018],
    #                         [ 0.6337458  , 4.77305607 , 1.8855901],
    #                         [ 0.43458885 , 5.35266432 , 2.0],
    #                         [ 0.306866032 , 5.75183712 , 2.00 ],
    #                         [ 0.18826014 , 6.32242313 , 2.00],
    #                         [ 0.00302554 , 6.6410368 , 1.48696418],
    #                         [ 0.      ,    7     ,    1.        ]])

    waypoints = np.array([  [ 0.          ,0.          ,1.        ],
                        [-0.81413407  ,1.1448905   ,1.23811428],
                        [-0.87425354,  2.59736626,  1.60894123],
                        [ 0.48557153  ,3.29086545,  1.65730888],
                        [ 0.8275638   ,3.26540295  ,1.72477018],
                        [ 0.9337458   ,4.77305607  ,1.77855901],
                        [ 0.13458885  ,5.35266432  ,1.93773985],
                        [-0.46866032  ,5.75183712  ,2.1512191 ],
                        [-0.73826014  ,7.32242313  ,2.14789654],
                        [-0.28302554  ,7.39410368  ,1.48696418],
                        [ 0.,          7.2,         1.        ]])

    waypoints = np.round(waypoints, 2)

    waypoints = waypoints[::-1]
                                

    # timetrash , MP = generate_trajectory.generate_trajectory_multiple_waypoints(waypoints, time_per_segment=3, dt=0.05)
    # timetrash , MP = traj_gen_with_yaw.generate_smooth_trajectory(waypoints)
    timetrash , MP = new_traj_gen.new_traj_gen(waypoints)

    return MP

# HELPERS
def takeoffHelper(tobj):
    attempt = 0
    TAKEOFF_TIMEOUT = 10
    MAX_ATTEMPT = 2
    takeoffdone = False
    while True:
        attempt += 1

        tobj.send_command_without_return("takeoff")
        start_time = time.time()

        while True:
            el_time = time.time() - start_time
            if tobj.get_distance_tof() > 60.0:
                takeoffdone = True
                print('Takeoff complete in seconds = ', el_time)
                break
            elif el_time > TAKEOFF_TIMEOUT:
                takeoffdone = False
                break
            else:
                # sleep for 1 second and check again
                time.sleep(1)
        
        if takeoffdone:
            break
        elif attempt>=MAX_ATTEMPT:
            break
            
    return takeoffdone
def get_status(sta,V_log):
    x= sta.get_speed_x()
    y = sta.get_speed_y()
    z = sta.get_speed_z()
    vel = [x,y,z,time.time()]
    V_log.append(vel)
    print(f"X Vel : {x}, Y Vel : {y}, Z Vel : {z},")

def save_log(data, filename):
    csv_file = filename
    with open (csv_file,'w',newline='') as file:
        writer = csv.writer(file)
        writer.writerows(data)

if __name__ == '__main__':

    trajectory = get_trajectory()


    ##FOR VELOCITY##
    x_desired = trajectory[0,:]
    y_desired = trajectory[1,:]
    z_desired = trajectory[2,:]

    vx_desired = trajectory[3,:]
    vy_desired = trajectory[4,:]
    vz_desired = trajectory[5,:]

    yaw_desired = trajectory[9,:]
    ###########################

    save_log(trajectory,filename='waypointsv2.csv')
    ## waypoints
    csv_file = 'waypointsv1.csv'
    with open (csv_file,'w',newline='') as file:
        writer = csv.writer(file)
        writer.writerows(trajectory)

    n = len(x_desired)-1

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_desired, y_desired, z_desired)
    ax.set_title('3D Position Plot')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_xlim([-1.8, 1.8])
    ax.set_ylim([-1.2, 7.4])
    ax.set_zlim([0,2])


    plt.show()
    # INIT TELLO CLASS
    tello = Tello(retry_count=1)
    V_log =[]

    # THE MAIN PROGRAM
    try:
        # ESTABLISH CONNECTION ------------------------------------------------------------------
        attempt = 0
        while True:
            try:
                # ENTER COMMAND MODE AND TRY CONNECTING OVER UDP
                attempt += 1
                print("Takeoff attempt ", attempt)
                tello.connect()
            except:
                print('Failed to connect or it connected but "ok" not received. Retrying...')
                if attempt > 1:
                    print('Failed to connect after multiple attempts')
                    exit(-1)
            else:
                # No exception 
                break

        # CHECK SENSOR READINGS------------------------------------------------------------------
        print('Altitude ', tello.get_distance_tof())
        print('Battery, ', tello.get_battery())

        # TAKEOFF--------------------------------------------------------------------------------
        
            #   technique 1 - Call tello.takeoff() - 
            #       in case the takeoff works but ok is not received, it will think that takeoff is incomplete
            #      tello.takeoff()

        # technique 2 - send takeoff command and dont expect a response. see if altitude crosses some preset value
        # if takeoffHelper(tello) == False:
        #     print('takeoff failed after multiple attempts')
        #     exit(-1)

        tello.takeoff()
        time.sleep(2)
        speed = 100
        for i in range(0,n):

            # velocity controller - Give Velocity in XYZ and Yaw
            # tello.send_rc_control(int((vx_desired[i]*100)),int((vy_desired[i]*100)), int((vz_desired[i])*100),int((yaw_desired[i])*100))
            tello.send_rc_control(int((vx_desired[i]*100)),int((vy_desired[i]*100)), int((vz_desired[i])*100),0)
            get_status(tello,V_log)
            # SLEEP / SCHEDULING LOGIC
            # time.sleep(0.043)
            time.sleep(0.01)

            # Send keep alive or some other command to keep tello on.
            #   Even if keep alive is sent, it can still shutdown once over heated
            # tello.send_command_without_return("keepalive")

            missionComplete = False
            if missionComplete:
                break
        save_log(V_log,filename = "v_logv_2.csv")
        tello.land()

    except KeyboardInterrupt:
        # HANDLE KEYBOARD INTERRUPT AND STOP THE DRONE COMMANDS
        print('keyboard interrupt')
        tello.emergency()
        tello.emergency()
        tello.end()





