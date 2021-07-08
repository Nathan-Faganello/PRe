import argparse
import configparser
import numpy as np
import burg_toolkit as burg
import trimesh as tr



def select_stable_pose(mesh):
    # among all the possible poses for the mesh, it returns the most probable
    # mesh = the mesh for which we need to have the stable pose
    trimesh = burg.util.o3d_mesh_to_trimesh(mesh)
    stable_poses, proba = tr.poses.compute_stable_poses(trimesh)
    N = len(stable_poses)
    proba_max = proba[0]
    index_max = 0
    for index in range (1,N):
        if (proba[index]>proba_max):
            proba_max = proba[index]
            index_max = index 
    return stable_poses[index_max]


def parse_args():
    parser = argparse.ArgumentParser(description='training on the simulation of burg toolkit')
    parser.add_argument('-c', '--config_fn', default='../config/config.cfg', type=str, metavar='FILE',
                        help='path to config file')
    return parser.parse_args()

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


if __name__ == "__main__":
    arguments = parse_args()
    cfg = configparser.ConfigParser()
    # read config file and use the section that contains the data paths
    cfg.read(arguments.config_fn)
    training_demo(cfg['General'])
