import argparse
import configparser
import numpy as np
import burg_toolkit as burg
import trimesh as tr



def generate_random_pose_around_coordinates(x, y, z):
    #translation to get around the aimed point
    T = np.eye(4)
    T[0,3] = x
    T[1,3] = y
    T[2,3] = z
    
    #random rotation aroud z
    # (i) random angle between 0 and 2*pi
    gamma = np.random.uniform(low = 0, high = 2*np.pi)
    cz, sz = np.cos(gamma), np.sin(gamma)
    Rz = np.array([cz, -sz, 0,0], [sz, cz, 0,0], [0,0,1,0],[0,0,0,1])

    #random rotation aroud y
    # (i) random angle between -pi/2 and pi/2
    beta = np.random.uniform(low = -np.pi/2, high = np.pi/2)
    cy, sy = np.cos(beta), np.sin(beta)
    Ry = np.array([cy, 0, sin,0], [0, 1, 0,0], [-sy,0,cy,0],[0,0,0,1])

    #random rotation aroud x
    # (i) random angle between -pi/2 and pi/2
    alpha = np.random.uniform(low = -np.pi/2, high = np.pi/2)
    cx, sx = np.cos(alpha), np.sin(alpha)
    Rx = np.array([1,0,0,0], [0, cx, -sx,0], [0,sx,cx,0],[0,0,0,1])

    pose = np.dot(T, np.dot(Rz, np.dot(Ry, Rx)))

    return pose

def generate_pose(x,y,z, alpha, beta, gamma):
    #translation to get around the aimed point
    T = np.eye(4)
    T[0,3] = x
    T[1,3] = y
    T[2,3] = z
    
    #random rotation aroud z
    cz, sz = np.cos(gamma), np.sin(gamma)
    Rz = np.array([cz, -sz, 0,0], [sz, cz, 0,0], [0,0,1,0],[0,0,0,1])

    #random rotation aroud y
    cy, sy = np.cos(beta), np.sin(beta)
    Ry = np.array([cy, 0, sin,0], [0, 1, 0,0], [-sy,0,cy,0],[0,0,0,1])

    #random rotation aroud x
    cx, sx = np.cos(alpha), np.sin(alpha)
    Rx = np.array([1,0,0,0], [0, cx, -sx,0], [0,sx,cx,0],[0,0,0,1])

    pose = np.dot(T, np.dot(Rz, np.dot(Ry, Rx)))

    return pose

def generate_set_poses(x, y, z, gap):
    #gap = angle between each rotation
    poses = []

    #alpha for x, beta for y, gamma for z
    alpha = -np.pi/2
    beta = -np.pi/2
    gamma = 0

    while gamma <= 2*np.pi:
        while beta < np.pi / 2:
            while alpha < np.pi / 2:
                pose = generate_pose(x,y,z,alpha, beta, gamma)
                poses += [pose]
                alpha += gap
            beta += gap
        gamma += gap
    return poses