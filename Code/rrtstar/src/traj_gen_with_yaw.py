import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculate_segment_times(waypoints):
    distances = np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1))
    min_time, max_time = 4.0 , 4.0  # Time in seconds for shortest and longest segments
    times = np.interp(distances, [np.min(distances), np.max(distances)], [min_time, max_time])
    return np.cumsum(np.insert(times, 0, 0))  # Cumulative time for each waypoint

def calculate_yaw_angles(waypoints, num_points=2000):
    waypoints = np.array(waypoints)
    segment_times = calculate_segment_times(waypoints)
    yaw_angles = np.arctan2(np.diff(waypoints[:, 1]), np.diff(waypoints[:, 0]))

    # Interpolating the yaw angles
    t_yaw = segment_times[:-1]
    spline_yaw = CubicSpline(t_yaw, yaw_angles, bc_type='natural')
    t_fine_yaw = np.linspace(0, segment_times[-1], num_points)

    return spline_yaw(t_fine_yaw)

def generate_smooth_trajectory(waypoints, num_points=2000):
    waypoints = np.array(waypoints)
    segment_times = calculate_segment_times(waypoints)
    t_fine = np.linspace(0, segment_times[-1], num_points)

    # Boundary conditions: zero velocity and acceleration at start
    bc_type = ((1, 0), (1, 0))  # (1, 0) means first derivative (velocity) set to 0 at both ends

    spline_x = CubicSpline(segment_times, waypoints[:, 0], bc_type=bc_type)
    spline_y = CubicSpline(segment_times, waypoints[:, 1], bc_type=bc_type)
    spline_z = CubicSpline(segment_times, waypoints[:, 2], bc_type=bc_type)

    position_trajectory = np.vstack((spline_x(t_fine), spline_y(t_fine), spline_z(t_fine))).T
    velocity_trajectory = np.vstack((spline_x.derivative()(t_fine), spline_y.derivative()(t_fine), spline_z.derivative()(t_fine))).T
    acceleration_trajectory = np.vstack((spline_x.derivative().derivative()(t_fine), spline_y.derivative().derivative()(t_fine), spline_z.derivative().derivative()(t_fine))).T

    interpolated_yaw_angles = calculate_yaw_angles(waypoints, num_points)
    yaw_velocities = np.gradient(interpolated_yaw_angles, t_fine)

    # Apply decay factor to the last segment
    start_last_segment = num_points - len(segment_times)
    decay_factor = np.linspace(1, 0, len(segment_times))**2
    velocity_trajectory[start_last_segment:] *= decay_factor[:, np.newaxis]
    yaw_velocities[start_last_segment:] *= decay_factor
    acceleration_trajectory[start_last_segment:] *= decay_factor[:, np.newaxis]

    px_trajectory = position_trajectory[:, 0] 
    py_trajectory = position_trajectory[:, 1]
    pz_trajectory = position_trajectory[:, 2]

    vx_trajectory = velocity_trajectory[:, 0] 
    vy_trajectory = velocity_trajectory[:, 1]
    vz_trajectory = velocity_trajectory[:, 2]

    ax_trajectory = acceleration_trajectory[:, 0] 
    ay_trajectory = acceleration_trajectory[:, 1]
    az_trajectory = acceleration_trajectory[:, 2]

    yaw_position_trajectory = interpolated_yaw_angles
    yaw_vel_trajectory = yaw_velocities 

    MP_trajectory_Matrix = np.array([[px_trajectory], [py_trajectory], [pz_trajectory],
                                     [vx_trajectory], [vy_trajectory], [vz_trajectory],
                                     [ax_trajectory], [ay_trajectory], [az_trajectory], [yaw_vel_trajectory]])
    MP_trajectory_Matrix = MP_trajectory_Matrix.reshape(MP_trajectory_Matrix.shape[0],MP_trajectory_Matrix.shape[2])
    

    return t_fine, MP_trajectory_Matrix

def plot_trajectory_profile(waypoints, segment_duration=3, dt=0.0375):
    # Generate trajectory
    time_trajectory, MP_trajectory_Matrix = generate_smooth_trajectory(waypoints, segment_duration, dt)

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