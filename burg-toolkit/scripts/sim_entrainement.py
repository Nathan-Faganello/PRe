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

def training_demo(data_conf):
    reader = burg.io.BaseviMatlabScenesReader(data_conf)

    print('read object library')
    object_library, index2name = reader.read_object_library()
    object_library.yell()
    
    print('generating urdf files for object library')
    object_library.generate_urdf_files('../data/tmp')

    target_object1 = object_library['table']
    mesh1 = target_object1.mesh
    stable_pose1 = select_stable_pose(mesh1)
    #burg.mesh_processing.check_properties(mesh1)
    #objectType1 = burg.scene.ObjectType(identifier = 'table', mesh = mesh1)
    target_object1.make_urdf_file(directory = '../data/tmp')
    obj_pose1 = np.array([
                [ -1, 0, 0, 0],
                [ 0, 1, 0, 0],
                [ 0, 0, 1, 0],
                [ 0, 0, 0 , 1]])
    pose1 = obj_pose1 @ stable_pose1
    objectInst1 = burg.scene.ObjectInstance(target_object1, pose = pose1)

    target_object2 = object_library['bowl']
    mesh2 = target_object2.mesh
    stable_pose2 = select_stable_pose(mesh2)
    #burg.mesh_processing.check_properties(mesh2)
    #objectType2 = burg.scene.ObjectType(identifier = 'bowl', mesh = mesh2, mass = 0.5)
    target_object2.make_urdf_file(directory = '../data/tmp')
    obj_pose2 = np.array([
                [ 1, 0, 0, 0.3],
                [ 0, 1, 0, 0.3],
                [ 0, 0, 1, 0],
                [ 0, 0, 0 , 1]])
    pose2 = obj_pose2 @ stable_pose2
    objectInst2 = burg.scene.ObjectInstance(target_object2, pose= pose2)

    target_object3 = object_library['softball']
    mesh3 = target_object3.mesh
    stable_pose3 = select_stable_pose(mesh3)
    #burg.mesh_processing.check_properties(mesh3)
    #objectType3 = burg.scene.ObjectType(identifier = 'softball', mesh = mesh3, mass = 0.1)
    target_object3.make_urdf_file(directory = '../data/tmp')
    obj_pose3 = np.array([
                [ 1, 0, 0, 0.3],
                [ 0, 1, 0, -0.3],
                [ 0, 0, 1, 0],
                [ 0, 0, 0 , 1]])
    pose3 = obj_pose3 @ stable_pose3
    objectInst3 = burg.scene.ObjectInstance(target_object3, pose = pose3)

    target_object4 = object_library['pear']
    mesh4 = target_object4.mesh
    stable_pose4 = select_stable_pose(mesh4)
    #burg.mesh_processing.check_properties(mesh4)
    #objectType4 = burg.scene.ObjectType(identifier = 'pear', mesh = mesh4, mass = 0.05)
    target_object4.make_urdf_file(directory = '../data/tmp')
    obj_pose4 = np.array([
                [ 1, 0, 0, 0.3],
                [ 0, 1, 0, 0.3],
                [ 0, 0, 1, 0],
                [ 0, 0, 0 , 1]])
    pose4 = obj_pose4 @ stable_pose4  
    objectInst4 = burg.scene.ObjectInstance(target_object4, pose = pose4)

    target_object5 = object_library['foamBrick']
    mesh5 = target_object5.mesh
    stable_pose5 = select_stable_pose(mesh5)
    #objectType5 = burg.scene.ObjectType(identifier = 'foamBrick', mesh = mesh5, mass = 0.05)
    target_object5.make_urdf_file(directory = '../data/tmp')
    obj_pose5 = np.array([
                [ 1, 0, 0, 0],
                [ 0, 1, 0, 0],
                [ 0, 0, 1, 0],
                [ 0, 0, 0 , 1]])
    pose5 = obj_pose5 @ stable_pose5   
    objectInst5 = burg.scene.ObjectInstance(target_object5, pose = pose5)

    objects_bg = [objectInst1]
    objects_scene = [objectInst2, objectInst3, objectInst4, objectInst5]
    sc = burg.scene.Scene(objects = objects_scene, bg_objects = objects_bg)


    gripper_model = burg.gripper.Robotiq2F85()
    grasp_pose = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    g = burg.grasp.Grasp()
    g.pose = grasp_pose

    # show grasp for mesh of object type
    burg.visualization.show_grasp_set([objectInst5.object_type.mesh], g, gripper=gripper_model)

    # transform grasp to pose of object instance and show that
    g.transform(objectInst5.pose)
    burg.visualization.show_grasp_set([objectInst5.get_mesh()], g, gripper=gripper_model, with_plane=False)

    # simulation should look like that as well
    #sim = burg.sim.SceneGraspSimulator(target_object = objectInst5, gripper= gripper_model, scene=sc, verbose=True)
    sim = burg.sim.SingleObjectGraspSimulator(target_object = objectInst5, gripper= gripper_model, verbose=True)
    sim.simulate_grasp_set(g)
    sim.dismiss()


def multiple_sim(data_conf):
    reader = burg.io.BaseviMatlabScenesReader(data_conf)

    print('read object library')
    object_library, index2name = reader.read_object_library()
    object_library.yell()
    
    print('generating urdf files for object library')
    object_library.generate_urdf_files('../data/tmp')

    # 1-load the object we want to grab
    target_object = object_library['foamBrick']
    mesh = target_object.mesh
    stable_pose = burg.mesh_processing.select_stable_pose(mesh)
    target_object.make_urdf_file(directory = '../data/tmp')
    objectInst = burg.scene.ObjectInstance(target_object, pose = stable_pose)

    #2-load the gripper
    gripper_model = burg.gripper.Robotiq2F85()
    grasp_pose = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    g = burg.grasp.Grasp()
    g.pose = grasp_pose


    #3 - starting from a basic position, creating a multitude of different grasp poses
    """ nb_poses = 1000
    grasp_set=np.array([])
    pi = np.pi
    interval = 0.01
    for nPose in range(nb_poses):
        pose = burg.util.tf_rotation_from_angle(rd.uniform(0, 2*pi),"z")@ burg.util.tf_rotation_from_angle(rd.uniform(-pi/2,pi/2),"y")@ burg.util.tf_rotation_from_angle(rd.uniform(-pi/2,pi/2),"x")
        height = rd.randint(0,10)
        pose = burg.util.tf_translation(interval * height, "z") @ pose
        g = burg.grasp.Grasp()
        g.pose = pose @ grasp_pose
        grasp_set = np.append(grasp_set, g) """
    
    grasp_set = create_antipodal_grasp_set(objectInst)
    
    sim = burg.sim.SingleObjectGraspSimulator(target_object = objectInst, gripper= gripper_model, verbose=True)
    scores = sim.simulate_grasp_set(grasp_set)
    sim.dismiss()

    #np.savetxt() we want to save the scores and the grasps in a csv file for learning
    
    

if __name__ == "__main__":
    arguments = parse_args()
    cfg = configparser.ConfigParser()
    # read config file and use the section that contains the data paths
    cfg.read(arguments.config_fn)
    #training_demo(cfg['General'])
    multiple_sim(cfg['General'])
