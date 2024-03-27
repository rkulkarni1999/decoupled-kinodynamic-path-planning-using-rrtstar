######################
# IMPORTING LIBRARIES
######################
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# FUNCTION FOR GENERATING TRAJECTORY FOR A SINGLE SEGMENT
def generate_segment_trajectory(waypoints, v0, a0, vf, af, trajectory_duration, dt):

    # Time Parameters
    t = np.array([0, trajectory_duration])                     # time limits for the execution of trajectory

    # Position Parameters

    # Extract all positions
    p0 = waypoints[0]
    pf = waypoints[1]
    # Extracting x,y and z seperately.
    p0_x, p0_y, p0_z = p0[0], p0[1], p0[2]
    pf_x, pf_y, pf_z = pf[0], pf[1], pf[2]

    ########################
    # Velocity Parameters (Fixed for now, will later be dependent on previous velocity and the distance heuristic)
    ########################
    # Extracting x,y and z seperately.
    v0_x, v0_y, v0_z = v0[0], v0[1], v0[2]
    vf_x, vf_y, vf_z = vf[0], vf[1], vf[2]

    ########################
    # Acceleration Parameters (Fixed for now, will later be dependent on the prevous acceleration)
    ########################
    # Extracting x,y and z seperately.
    a0_x, a0_y, a0_z = a0[0], a0[1], a0[2]
    af_x, af_y, af_z = af[0], af[1], af[2]

    ##########################
    # Coefficient Matrix (A)
    ##########################
    # Assuming t and q, v, a are defined
    A = np.array([[1, t[0], t[0]**2, t[0]**3, t[0]**4, t[0]**5],
                    [0, 1, 2*t[0], 3*t[0]**2, 4*t[0]**3, 5*t[0]**4],
                    [0, 0, 2, 6*t[0], 12*t[0]**2, 20*t[0]**3],
                    [1, t[1], t[1]**2, t[1]**3, t[1]**4, t[1]**5],
                    [0, 1, 2*t[1], 3*t[1]**2, 4*t[1]**3, 5*t[1]**4],
                    [0, 0, 2, 6*t[1], 12*t[1]**2, 20*t[1]**3]])

    ##############################################
    # Initial Condition Matrix (Q) for x,y and z
    ##############################################
    q_x = np.array([p0_x, v0_x, a0_x, pf_x, vf_x, af_x])
    q_y = np.array([p0_y, v0_y, a0_y, pf_y, vf_y, af_y])
    q_z = np.array([p0_z, v0_z, a0_z, pf_z, vf_z, af_z])

    # Reshaping the Matrix into a Column Vector.
    q_x = np.reshape(q_x, (6,1))
    q_y = np.reshape(q_y, (6,1))
    q_z = np.reshape(q_z, (6,1))

    ################################
    # Solving for Coefficients
    ################################
    coeffs_x = np.linalg.solve(A,q_x)
    coeffs_y = np.linalg.solve(A,q_y)
    coeffs_z = np.linalg.solve(A,q_z)

    # Flatten the coeffs
    coeffs_x_flat = coeffs_x.flatten()
    coeffs_y_flat = coeffs_y.flatten()
    coeffs_z_flat = coeffs_z.flatten()

    ##################################
    # Calculating the number of Steps
    ##################################
    n = int((t[1] - t[0])/dt)

    #########################################
    # Initializing Trajectories in x,y and z
    #########################################

    # Time Trajectory
    time_trajectory = [0]

    # Position Trajectory
    px_trajectory = [coeffs_x_flat[0]]
    py_trajectory = [coeffs_y_flat[0]]
    pz_trajectory = [coeffs_z_flat[0]]

    # Velocity Trajectory
    vx_trajectory = [coeffs_x_flat[1]]
    vy_trajectory = [coeffs_y_flat[1]]
    vz_trajectory = [coeffs_z_flat[1]]

    # Acceleration Trajectory
    ax_trajectory = [2*coeffs_x_flat[2]]
    ay_trajectory = [2*coeffs_y_flat[2]]
    az_trajectory = [2*coeffs_z_flat[2]]

    for ii in range(n):

        # Updating Time Trajectory
        time = time_trajectory[-1] + dt
        time_trajectory.append(round(time,2))

        # Calculating Position Trajectory.
        position_x = np.dot([1, time, time**2, time**3, time**4, time**5], coeffs_x_flat)
        position_y = np.dot([1, time, time**2, time**3, time**4, time**5], coeffs_y_flat)
        position_z = np.dot([1, time, time**2, time**3, time**4, time**5], coeffs_z_flat)

        # Updating Position Trajectory
        px_trajectory.append(position_x)
        py_trajectory.append(position_y)
        pz_trajectory.append(position_z)

        # Calculating Velocity Trajectory.
        velocity_x = np.dot([0, 1, 2*time, 3*time**2, 4*time**3, 5*time**4], coeffs_x_flat)
        velocity_y = np.dot([0, 1, 2*time, 3*time**2, 4*time**3, 5*time**4], coeffs_y_flat)
        velocity_z = np.dot([0, 1, 2*time, 3*time**2, 4*time**3, 5*time**4], coeffs_z_flat)

        # Updating Velocities
        vx_trajectory.append(velocity_x)
        vy_trajectory.append(velocity_y)
        vz_trajectory.append(velocity_z)

        # Calculating Acceleration Trajectory.
        acceleration_x = np.dot([0, 0, 2, 6*time, 12*time**2, 20*time**3], coeffs_x_flat)
        acceleration_y = np.dot([0, 0, 2, 6*time, 12*time**2, 20*time**3], coeffs_y_flat)
        acceleration_z = np.dot([0, 0, 2, 6*time, 12*time**2, 20*time**3], coeffs_z_flat)

        # Updating Accelerations
        ax_trajectory.append(acceleration_x)
        ay_trajectory.append(acceleration_y)
        az_trajectory.append(acceleration_z)

    time_trajectory = time_trajectory[1:]

    px_trajectory = px_trajectory[1:]
    py_trajectory = py_trajectory[1:]
    pz_trajectory = pz_trajectory[1:]
    vx_trajectory = vx_trajectory[1:]
    vy_trajectory = vy_trajectory[1:]
    vz_trajectory = vz_trajectory[1:]
    ax_trajectory = ax_trajectory[1:]
    ay_trajectory = ay_trajectory[1:]
    az_trajectory = az_trajectory[1:]

    return time_trajectory,\
            px_trajectory, py_trajectory, pz_trajectory,\
            vx_trajectory, vy_trajectory, vz_trajectory,\
            ax_trajectory, ay_trajectory, az_trajectory


# FUNCTION FOR GENERATING TRAJECTORY FOR MULTIPLE SEGMENTS
def generate_trajectory_multiple_waypoints(waypoints, time_per_segment = 3, dt = 0.0375):
    # length = len(SET)+1
    # # waypoints = np.zeros((length,3))
    # waypoints =[]
    # count = 0
    # print(SET)
    # for i in SET:
    #     # waypoints[count] =  np.array([i[0][0],i[0][1],i[0][2]])
    #     # count = count +1
    #     point = np.array([i[0][0],i[0][1],i[0][2]])
    #     waypoints.append(point)
    # i = SET[-1]
    # # waypoints[count] = np.array([i[1][0],i[1][1],i[1][2]])
    # waypoints.append(np.array([i[1][0],i[1][1],i[1][2]]))
    # # print(waypoints)
    print("inside trajectory")
    print(waypoints)
    segment_duration = time_per_segment

    # Final time trajectory
    final_time_trajectory = [0]

    # Position
    final_px_trajectory = []
    final_py_trajectory = []
    final_pz_trajectory = []
    # Velocity
    final_vx_trajectory = []
    final_vy_trajectory = []
    final_vz_trajectory = []
    # Acceleration
    final_ax_trajectory = []
    final_ay_trajectory = []
    final_az_trajectory = []

    # Iterate Each Pair of Waypoints.
    for ii in range(len(waypoints)-1):

        # Set the initial and final waypoints for this segment.
        segment_waypoints = np.array([waypoints[ii], waypoints[ii+1]])

        # If this is the first segment, set initial velocity and acceleration to 0.
        if ii == 0:
            v0 = np.array([0.0, 0.0, 0.0])
            a0 = np.array([0.0, 0.0, 0.0])

        # Otherwise set the initial velocity and acceleration to the final values of the last segment.
        else:
            v0 = np.array([final_vx_trajectory[-1], final_vy_trajectory[-1], final_vz_trajectory[-1]])
            a0 = np.array([final_ax_trajectory[-1], final_ay_trajectory[-1], final_az_trajectory[-1]])

        # If this is the last segment, set the final velocity and acceleration to zero.
        if ii == len(waypoints) - 2:
            vf = np.array([0.0, 0.0, 0.0])
            af = np.array([0.0, 0.0, 0.0])

        # Otherwise Calculate the Final Velocity and Acceleration based on the next waypoint.
        else:
            vf = (waypoints[ii+2] - waypoints[ii+1]) / segment_duration    # distance / time
            af = (vf - v0) / segment_duration                            # velocity / time

        ################################################################
        # Using make_trajectory function to generate segment trajectory
        ################################################################
        time_trajectory,\
        px_trajectory, py_trajectory,pz_trajectory,\
        vx_trajectory, vy_trajectory,vz_trajectory,\
        ax_trajectory, ay_trajectory, az_trajectory = generate_segment_trajectory(segment_waypoints, v0, a0, vf, af, segment_duration, dt)

        ############################################################################################
        # Append the position, velocity and acceleration of this trajectory to the final trajectory
        ############################################################################################
        # Position
        final_px_trajectory = final_px_trajectory + px_trajectory
        final_py_trajectory = final_py_trajectory + py_trajectory
        final_pz_trajectory = final_pz_trajectory + pz_trajectory
        # Velocity
        final_vx_trajectory = final_vx_trajectory + vx_trajectory
        final_vy_trajectory = final_vy_trajectory + vy_trajectory
        final_vz_trajectory = final_vz_trajectory + vz_trajectory
        # Acceleration
        final_ax_trajectory = final_ax_trajectory + ax_trajectory
        final_ay_trajectory = final_ay_trajectory + ay_trajectory
        final_az_trajectory = final_az_trajectory + az_trajectory

        ################################
        # Work-around on Updating Time
        ################################
        for jj in range(int(segment_duration/dt)):
            incremented_time = final_time_trajectory[-1] + dt
            final_time_trajectory.append(round(incremented_time,2))

    # Slice the time trajectory to match timestamps.
    final_time_trajectory = final_time_trajectory[1:]

    # Storing Everything in a Final Matrix. 
    MP_trajectory_Matrix = np.array([[final_px_trajectory], [final_py_trajectory], [final_pz_trajectory],
                                     [final_vx_trajectory], [final_vy_trajectory], [final_vz_trajectory],
                                     [final_ax_trajectory], [final_ay_trajectory], [final_az_trajectory]])
    MP_trajectory_Matrix = MP_trajectory_Matrix.reshape(MP_trajectory_Matrix.shape[0],MP_trajectory_Matrix.shape[2])
    # plot_trajectory_profile(waypoints, segment_duration=3, dt=0.0375)
    return final_time_trajectory, MP_trajectory_Matrix

############################################################
# FUNCTION FOR PLOTTING THE TRAJECTORIES USING MATPLOTLIB
############################################################
def plot_trajectory_profile(waypoints, segment_duration=3, dt=0.0375):
    # Generate trajectory
    time_trajectory, MP_trajectory_Matrix = generate_trajectory_multiple_waypoints(waypoints, segment_duration, dt)

    # Plot velocity
    plt.figure()
    plt.plot(time_trajectory, MP_trajectory_Matrix[3, :], label='Vx')
    plt.plot(time_trajectory, MP_trajectory_Matrix[4, :], label='Vy')
    plt.plot(time_trajectory, MP_trajectory_Matrix[5, :], label='Vz')
    plt.title('Velocity Profile')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid(True)

    # Plot acceleration
    plt.figure()
    plt.plot(time_trajectory, MP_trajectory_Matrix[6, :], label='Ax')
    plt.plot(time_trajectory, MP_trajectory_Matrix[7, :], label='Ay')
    plt.plot(time_trajectory, MP_trajectory_Matrix[8, :], label='Az')
    plt.title('Acceleration Profile')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()
    plt.grid(True)

    # Plot 3D position
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(MP_trajectory_Matrix[0, :], MP_trajectory_Matrix[1, :], MP_trajectory_Matrix[2, :])
    ax.set_title('3D Position Plot')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')

    plt.show()





if __name__ == '__main__':
    pass
    # waypoints = np.array([(5.0, 17.0, 2.0),
    # (4.707526804631064, 16.902793925191784, 2.8502401255179888),
    # (4.849188783948092, 16.741662013709877, 3.222247549957932),
    # (4.951386170891119, 16.04073905906404, 3.032313835842028),
    # (5.445143902341972, 14.892361896641646, 3.1526991196254563),
    # (6.164776064462577, 14.350182592362675, 3.3365004389977466),
    # (6.254596793191187, 12.36063841706286, 3.5199294749251027),
    # (6.344417521919796, 10.371094241763045, 3.7033585108524587),
    # (5.788618796827702, 8.449874254115448, 3.7045979661257467),
    # (5.744481808865578, 6.455781218608851, 3.8517199972350165),
    # (5.6610733100610435, 4.822224274287095, 3.4096983352745753),
    # (5.154237321479032, 3.6522850922512546, 3.4237601728006934),
    # (5.4276306366022276, 1.9810337838489636, 2.4982933677131633),
    # (5.113444716269026, 0.5772268419826383, 1.135188248135702),
    # (5.151831117951446, -0.6634959328182237, 0.9301579099364596),
    # (5.431446820512459, -1.69899511954097, 0.11304481629071139),
    # (5.0, -3.0, 0.0)])

    # Generating trajectory
    # time_trajectory,\
    # MP_trajectory_Matrix = generate_trajectory_multiple_waypoints(waypoints, time_per_segment=3, dt=0.0375)

    # Plotting Trajectory
    # plot_trajectory_profile(waypoints, segment_duration=3, dt=0.0375)