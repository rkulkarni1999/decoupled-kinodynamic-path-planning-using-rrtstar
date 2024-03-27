
import sys
import site
import csv
import threading
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
# import cv2
import scipy
import rrt_star3D
import time
import generate_trajectory
#import OpenEXR

import usercode
from djitellopy import Tello

importlib.reload(usercode)
import matplotlib.pyplot as plt
import matplotlib
# matplotlib.use("TkAgg")

def get_trajectory():
    p = rrt_star3D.rrtstar()
    endpoints = p.run()
    length = len(endpoints)+1

    waypoints = np.zeros((length,3))
    count = length -1
    for i in endpoints:
        waypoints[count] =  np.array([i[0][0],i[0][1],i[0][2]])
        count = count - 1
    i = endpoints[-1]
    waypoints[0] = np.array([i[1][0],i[1][1],i[1][2]])
    return waypoints

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

def send_commandtofly(tello,xi,x,yi,y,zi,z,speed):
    cmd = 'go {} {} {} {}'.format(int((xi-x)*100), int((yi-y)*100), int((zi-z)*100), speed)
    tello.send_control_command(cmd)
    time.sleep(0.042)

def save_log(data, filename):
    csv_file = filename
    with open (csv_file,'w',newline='') as file:
        writer = csv.writer(file)
        writer.writerows(data)

if __name__ == '__main__':

    trajectory = get_trajectory()

    # ##FOR POSITION##
    x_desired = trajectory[:,0]
    y_desired = trajectory[:,1]
    z_desired = trajectory[:,2]

    # x_desired = [0,0,0.5,1]
    # y_desired = [0,0.5,1,1.5]
    # z_desired = [0,0,0,0]

    # save_log(trajectory,filename='waypointsp3.csv')
    
    # waypoints
    # csv_file = 'waypoints2.csv'
    # with open (csv_file,'w',newline='') as file:
    #     writer = csv.writer(file)
    #     writer.writerows(trajectory)

    n = len(x_desired)-1

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_desired, y_desired, z_desired)
    ax.set_title('3D Position Plot')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')

    plt.show()
    # INIT TELLO CLASS
    tello = Tello(retry_count=1)
    V_log = []
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
        time.sleep(3)
        speed = 100
        for i in range(0,n):
            

            velocity_th = threading.Thread(target=get_status, args=(tello,V_log))
            velocity_th.start()
            # position control
            # fly_th = threading.Thread(target=send_commandtofly, args=(tello,x_desired[i+1],x_desired[i],y_desired[i+1],y_desired[i],z_desired[i+1],z_desired[i],speed))
            cmd = 'go {} {} {} {}'.format(int((x_desired[i+1]-x_desired[i])*100), int((y_desired[i+1]-y_desired[i])*100), int((z_desired[i+1]-z_desired[i])*100), speed)
            tello.send_control_command(cmd)
            print("command sent")
            time.sleep(0.05)
            # velocity_th.start()
            # fly_th.start()

            # Send keep alive or some other command to keep tello on.
            #   Even if keep alive is sent, it can still shutdown once over heated
            # tello.send_command_without_return("keepalive")

            missionComplete = False
            if missionComplete:
                break
        # velocity_th.join()
        # fly_th.join()
        save_log(V_log,filename = "v_logp_3.csv")
        tello.land()
        # save_log(V_log,filename = "v_log_3.csv")

    except KeyboardInterrupt:
        # HANDLE KEYBOARD INTERRUPT AND STOP THE DRONE COMMANDS
        print('keyboard interrupt')
        tello.emergency()
        tello.emergency()
        tello.end()





