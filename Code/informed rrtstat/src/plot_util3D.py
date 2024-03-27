# plotting
import numpy as np
import math
import blender_plots as bplt
from scipy.spatial.transform import Rotation
import bpy


def calculate_euler_angles(endpoint2, endpoint1):
    # Normalize the rotated vector
    rotated_vector = np.array(endpoint2) - np.array(endpoint1)
    rotated_vector = np.array(rotated_vector) / np.linalg.norm(rotated_vector)

    # Calculate the rotation matrix that aligns the Z-axis with the rotated vector
    z_axis = np.array([0, 0, 1])
    rotation_matrix = np.eye(3)
    rotation_matrix[:, 2] = rotated_vector
    rotation_matrix[:, 0] = np.cross(rotated_vector, z_axis)
    rotation_matrix[:, 0] /= np.linalg.norm(rotation_matrix[:, 0])
    rotation_matrix[:, 1] = np.cross(rotation_matrix[:, 2], rotation_matrix[:, 0])

    # Use the rotation matrix to calculate Euler angles in XYZ format
    r = Rotation.from_matrix(rotation_matrix)
    euler_angles_xyz = r.as_euler('xyz', degrees=False)  # XYZ rotation order

    return euler_angles_xyz

def draw_line(SET,initparams, colour=None,):
    length = len(SET)
    eulerangles = np.zeros((length,3))
    cylinder_length = np.zeros((length,1))
    cylinder_center = np.zeros((length,3))
    if SET != []:
        count = 0
        for i in SET:
            endpoint2 = [i[0][0],i[0][1],i[0][2]]
            endpoint1 = [i[1][0],i[1][1],i[1][2]]
            cylinder_length[count] = math.dist(endpoint1, endpoint2)
            cylinder_center[count] = ((endpoint1[0] + endpoint2[0]) / 2, (endpoint1[1] + endpoint2[1]) / 2, (endpoint1[2] + endpoint2[2]) / 2)
            eulerangles[count] = calculate_euler_angles(endpoint1, endpoint2)
            count = count +1
        
        # sca_name = f"sactter{initparams.ind}"
        for i in range(length):
            sca_name = f"sactter{i}"
            scatter = bplt.Scatter(
            cylinder_center[i],
            color=colour,
            marker_type="cylinders",
            marker_rotation = eulerangles[i],
            vertices=32,
            radius=0.05,
            depth=cylinder_length[i],
            name = sca_name,
                )
            bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    
def draw_edge(SET,initparams, colour=None,):
    length = len(SET)
    eulerangles = np.zeros((length,3))
    cylinder_length = np.zeros((length,1))
    cylinder_center = np.zeros((length,3))
    if SET != []:
        count = 0
        for i in SET:
            endpoint2 = [i[0][0],i[0][1],i[0][2]]
            endpoint1 = [i[1][0],i[1][1],i[1][2]]
            cylinder_length[count] = math.dist(endpoint1, endpoint2)
            cylinder_center[count] = ((endpoint1[0] + endpoint2[0]) / 2, (endpoint1[1] + endpoint2[1]) / 2, (endpoint1[2] + endpoint2[2]) / 2)
            eulerangles[count] = calculate_euler_angles(endpoint1, endpoint2)
            count = count +1
        
        # sca_name = f"sactter{initparams.ind}"
        print(cylinder_center)
        sca_name = 'scatter'
        scatter = bplt.Scatter(
        cylinder_center,
        color=colour,
        marker_type="cylinders",
        marker_rotation = eulerangles,
        vertices=32,
        radius=0.01,
        depth=0.8,
        name = sca_name,
            )
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)        

        # print(x)
def draw_node(SET,initparams, colour=None,):
    length = len(SET)
    node_centre = np.zeros((length,3))
    if SET != []:
        count = 0
        for i in SET:
            node_centre[count] = np.array([i[0][0],i[0][1],i[0][2]])
            count = count +1
    print(node_centre)
    sca_name = 'node'
    scatter = bplt.Scatter(
            node_centre,
            color=colour,
            marker_type="ico_spheres",
            radius=0.12,
            name = sca_name,
            )

def draw_node2(SET,initparams, colour=None,):
    length = len(SET)
    node_centre = np.zeros((length,3))
    if SET != []:
        count = 0
        for i in SET:
            node_centre[count] = np.array([i[0][0],i[0][1],i[0][2]])
            count = count +1
    print(node_centre)
    sca_name = 'scatter'
    scatter = bplt.Scatter(
            node_centre,
            color=colour,
            marker_type="ico_spheres",
            radius=0.13,
            name = sca_name,
            )


def visualization(initparams):
    if initparams.ind % 500 == 0 :
        #----------- list structure
        # V = np.array(list(initparams.V))
        # E = initparams.E
        #----------- end
        # edges = initparams.E
        Path = np.array(initparams.Path)
        start = initparams.env.start
        goal = initparams.env.goal
        # edges = E.get_edge()
        #----------- list structure
        edges = []
        for i in initparams.Parent:
            edges.append([i,initparams.Parent[i]])
        print("1")
        draw_edge(edges,initparams, colour=[0,128,0])
        draw_node(Path,initparams, colour=[128,0,0])
        draw_line(Path,initparams, colour=[0,0,128])
        draw_node(Path,initparams, colour=[128,0,0])
    if initparams.ind == 4000 :
        draw_node2(Path,initparams, colour=[0,128,0])


if __name__ == '__main__':
    pass
