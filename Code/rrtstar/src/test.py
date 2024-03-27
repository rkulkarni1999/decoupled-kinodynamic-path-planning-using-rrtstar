import numpy as np

x_desired = np.array([0,0,0,0])
y_desired = np.array([1,1.5,2,2.5])
z_desired = np.array([0,0,0,0])

n = 3
for i in range(0,n):

    # READ IMAGE

    # SEND CONTROL ACTION
    # cmd = f"rc {rightVel[wpcount]} {frontVel[wpcount]} 0 0"
    # tello.send_command_without_return(cmd)
    x = int(x_desired[i+1]-x_desired[i])*100
    y = int((y_desired[i+1]-y_desired)*100)
    z = int((z_desired[i+1]-z_desired)*100)
    cmd = 'go {} {} {} {}'.format(int((x_desired[i+1]-x_desired[i])*100), int((y_desired[i+1]-y_desired[i])*100), int((z_desired[i+1]-z_desired[i])*100), speed)
    print("command sent")
    tello.send_control_command(cmd)
    get_status(tello)
    # SLEEP / SCHEDULING LOGIC
    time.sleep(1)