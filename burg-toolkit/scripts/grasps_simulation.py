import os
import argparse
import configparser
import numpy as np
import burg_toolkit as burg
import trimesh as tr
import random as rd

from scripts import scene_creation


def parse_args():
    parser = argparse.ArgumentParser(description='training on the simulation of burg toolkit')
    parser.add_argument('-c', '--config_fn', default='../config/config.cfg', type=str, metavar='FILE',
                        help='path to config file')
    return parser.parse_args()

def create_antipodal_grasp_set(object):
    """
    Create a grasp set for a given object
    :object : (objectInstance) object you want to grab
    :return a grasp set (graspSet) 
    """
    gripper_model = burg.gripper.Robotiq2F85()

    ags = burg.sampling.AntipodalGraspSampler()
    ags.only_grasp_from_above = True
    ags.mesh = object.object_type.mesh
    ags.gripper = gripper_model
    ags.n_orientations = 18
    ags.verbose = True
    ags.max_targets_per_ref_point = 2
    graspset, contacts = ags.sample(100)
    # gs.scores = ags.check_collisions(gs, use_width=False)  # need to install python-fcl
    print('contacts.shape', contacts.shape)
    burg.visualization.show_grasp_set([ags.mesh], graspset, gripper=gripper_model, use_width=False,
                                      score_color_func=lambda s: [s, 1-s, 0], with_plane=True)

    return graspset


def sim_grasp_set_row(scene):
    """
    simulate grasps for a given scene
    :scene : scene in which we want to test an object grab
    :return : list of successful grasps
    """
    gripper_model = burg.gripper.Robotiq2F85()

    target_object = None

    found = False
    while found == False:
        print([obj.object_type.identifier for obj in scene.objects])
        buffer_target_object = input("Wich object in the previous list do you want to grab ?")

        i = 0
        while i < len(scene.objects):
            if (scene.objects[i].object_type.identifier == buffer_target_object):
                target_object = scene.objects[i]
                found = True
                break
            i += 1
        if found == False :
            print("the selected object is not in the list")

    grasp_set = create_antipodal_grasp_set(target_object)
    sim = burg.sim.SceneGraspSimulator(target_object = target_object, gripper= gripper_model, scene=scene, verbose=True)
    scores = sim.simulate_grasp_set(grasp_set)
    successful_grasps = []
    for index in range(len(scores)):
        if scores[index] == 5:
            successful_grasps += [grasp_set[index].as_grasp_set()]

    return successful_grasps


if __name__ == "__main__":
    arguments = parse_args()
    cfg = configparser.ConfigParser()
    # read config file and use the section that contains the data paths
    cfg.read(arguments.config_fn)
    sc = scene_creation.scene_creation_random(cfg['General'])
    sim_grasp_set_row(sc)