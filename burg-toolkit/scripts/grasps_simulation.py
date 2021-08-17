import os
import argparse
import configparser
import numpy as np
import burg_toolkit as burg
import trimesh as tr
import random as rd
import time

from scripts import scene_creation
from scripts import perturbations
from scripts import quality


def parse_args():
    parser = argparse.ArgumentParser(description='training on the simulation of burg toolkit')
    parser.add_argument('-c', '--config_fn', default='../config/config.cfg', type=str, metavar='FILE',
                        help='path to config file')
    return parser.parse_args()

def create_antipodal_grasp_set(object):
    """
    Create a grasp set for a given object
    :object : (objectInstance) object you want to grab
    :return a grasp set (graspSet) with, for each grasp, the associated contact points (N,2,3), normals (N,2,3) and the approach vectors (N,2,3) between normals and the approach direction of the finger
    """
    gripper_model = burg.gripper.Robotiq2F85()

    ags = burg.sampling.AntipodalGraspSampler()
    ags.only_grasp_from_above = True
    ags.mesh = object.object_type.mesh
    ags.gripper = gripper_model
    ags.n_orientations = 10
    ags.verbose = True
    ags.max_targets_per_ref_point = 2
    ags.no_contact_below_z = 0.01
    graspset, contacts, normals, approach_vectors = ags.sample(100)
    # gs.scores = ags.check_collisions(gs, use_width=False)  # need to install python-fcl
    print('contacts.shape', contacts.shape)
    #burg.visualization.show_grasp_set([ags.mesh], graspset, gripper=gripper_model, use_width=False,
    #                                  score_color_func=lambda s: [s, 1-s, 0], with_plane=True)
    
    """ print("graspset :", len(graspset))
    print("contacts :", contacts.shape)
    print("normales :", normals.shape)
    print("angles :", angles.shape) """

    return graspset, contacts, normals, approach_vectors


def create_antipodal_noisy_grasp_set(object, ref_grasp, contact_point):
    """
    Create a noised grasp set for a given object and a reference grasp
    :object : (objectInstance) object you want to grab
    :ref_grasp (grasp) : grasp we want to noise
    :contact_point(3,1) : one of the two contact points used as a reference
    :return a grasp set (graspSet) with, for each grasp, the associated contact points (N,2,3), normals (N,2,3) and the approach vectors (N,2,3) between normals and the approach direction of the finger
    """
    gripper_model = burg.gripper.Robotiq2F85()

    ags = burg.sampling.AntipodalGraspSampler()
    ags.only_grasp_from_above = True
    ags.mesh = object.object_type.mesh
    ags.gripper = gripper_model
    ags.n_orientations = 18
    ags.verbose = True
    ags.max_targets_per_ref_point = 2
    ags.no_contact_below_z = 0.01
    graspset, contacts, normals, approach_vectors = ags.sample_noisy(ref_grasp, contact_point, n=10)
    # gs.scores = ags.check_collisions(gs, use_width=False)  # need to install python-fcl
    print('contacts.shape', contacts.shape)
    #burg.visualization.show_grasp_set([ags.mesh], graspset, gripper=gripper_model, use_width=False,
    #                                  score_color_func=lambda s: [s, 1-s, 0], with_plane=True)
    
    """ print("graspset :", len(graspset))
    print("contacts :", contacts.shape)
    print("normales :", normals.shape)
    print("approach_vectors :", approach_vectors)
    """

    return graspset, contacts, normals, approach_vectors



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
        buffer_target_object = input("Wich object in the previous list do you want to grab ? ")

        i = 0
        while i < len(scene.objects):
            if (scene.objects[i].object_type.identifier == buffer_target_object):
                target_object = scene.objects[i]
                found = True
                break
            i += 1
        if found == False :
            print("the selected object is not in the list")

    #test a grasp set and create a list of successful grasp
    grasp_set, contact_points, normals, approach_vectors = create_antipodal_grasp_set(target_object)
    sim = burg.sim.SceneGraspSimulator(target_object = target_object, gripper= gripper_model, scene=scene, verbose=False)
    scores = sim.simulate_grasp_set(grasp_set)
    successful_grasps = burg.grasp.GraspSet()
    index_successfull = []
    params_successfull = []
    for index in range(len(scores)):
        if scores[index] == 5:
            successful_grasps.add(grasp_set[index].as_grasp_set())
            index_successfull += [index]
            params_successfull+=[[contact_points[index], normals[index], approach_vectors[index]]]
    sim.dismiss()

    print(len(successful_grasps))
    print("hellllo")

    time.sleep(10)

    #Evaluate each successfull grasp
    sim2 = burg.sim.SceneGraspSimulator(target_object = target_object, gripper= gripper_model, scene=scene, verbose=False)
    results = []
    for i, grasp in enumerate(successful_grasps):
        n_graspset, n_contact_points, n_normals, n_approach_vectors = create_antipodal_noisy_grasp_set(target_object, grasp, params_successfull[i][0][0])
        #n_graspset = perturbations.generate_perturb_grasp_set(grasp = grasp, nb_grasps = 50)
        #burg.visualization.show_grasp_set(objects = [target_object.object_type.mesh], gs = n_graspset, gripper = gripper_model, with_plane = True)
        params_noised = []
        for k in range(len(n_contact_points)):
            params_noised += [[n_contact_points[i], n_normals[i], n_approach_vectors[i]]]
        scores = sim2.simulate_grasp_set(n_graspset)
        robust = sum(scores)/len(scores)
        #metric = quality.probability_force_closure(n_graspset, params_noised)
        metric = quality.epsilon_quality(grasp, params_successfull[i] )
        results += [[grasp, robust, metric]]
        print([grasp, robust, metric])
        #results+= [[grasp, metric]]
        #results += [metric]

    print(results)
    sim2.dismiss()
    return results


if __name__ == "__main__":
    arguments = parse_args()
    cfg = configparser.ConfigParser()
    # read config file and use the section that contains the data paths
    cfg.read(arguments.config_fn)
    sc = scene_creation.scene_creation_random(cfg['General'])
    successful_grasp = sim_grasp_set_row(sc)