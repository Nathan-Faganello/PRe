import os
import argparse
import configparser
import numpy as np
import burg_toolkit as burg
import trimesh as tr
import random as rd

def parse_args():
    parser = argparse.ArgumentParser(description='training on the simulation of burg toolkit')
    parser.add_argument('-c', '--config_fn', default='../config/config.cfg', type=str, metavar='FILE',
                        help='path to config file')
    return parser.parse_args()

def generate_perturb_grasp_set(grasp, nb_grasps = 15):
    """
    From a successful grasp, generate a set of noised grasp
    :grasp : (Grasp)
    :return (graspSet)
    """

    noised_grasp_set = burg.grasp.GraspSet()

    for counter in range(nb_grasps):
        noised_grasp = generate_random_perturb_grasp(grasp)
        noised_grasp_set = noised_grasp_set.add(noised_grasp)
    
    return noised_grasp_set


def generate_random_perturb_grasp(grasp):
    """
    Basic perturbation tool
    From a successful grasp, create a noised grasp
    :grasp : successful grasp used as a model
    :return (GraspSet) :a single grasp (GraspSet of length 1)
    """
    """ we define a certain range for each noised parameter :
    x,y,z in [-0.1,0.1]
    if we call alpha, beta, gamma the angle around x,y,z, we have alpha, beta, gamma in [-20°,20°] """


    #define the range of the noise for each dimension (we'll probably be refined)
    transl_range = 0.1
    rot_range =  20 * np.pi / 180 

    #calculate the exact noise
    x_transl = rd.uniform(-transl_range, transl_range)
    y_transl = rd.uniform(-transl_range, transl_range)
    z_transl = rd.uniform(-transl_range, transl_range)

    alpha = rd.uniform(-rot_range, rot_range)
    beta = rd.uniform(-rot_range, rot_range)
    gamma = rd.uniform(-rot_range, rot_range)

    
    #create the correspondant transformation matrix
    tf = burg.util.tf_rotation_from_angle(alpha,"x") @ burg.util.tf_rotation_from_angle(beta,"y") @ burg.util.tf_rotation_from_angle(gamma,"z") @ burg.util.tf_translation(x_transl, "x") @ burg.util.tf_translation(y_transl, "y") @ burg.util.tf_translation(z_transl, "z")
    noised_pose = tf @ grasp.pose
    noised_grasp = burg.grasp.Grasp()
    noised_grasp.pose = noised_pose


    return noised_grasp.as_grasp_set()


def generate_noisy_graspset_information(object, graspset):

    ags = burg.sampling.AntipodalGraspSampler()
    ags.mesh = object.object_type.mesh

    



if __name__ == "__main__":
    arguments = parse_args()
    cfg = configparser.ConfigParser()
    # read config file and use the section that contains the data paths
    cfg.read(arguments.config_fn)

    base_pose = np.eye(4)
    base_grasp = burg.grasp.Grasp()
    base_grasp.pose = base_pose

    perturb_grasp_set = generate_perturb_grasp_set(base_grasp, 15)