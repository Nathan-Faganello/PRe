import argparse
import configparser
import numpy as np
import burg_toolkit as burg

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
    #burg.mesh_processing.check_properties(mesh1)
    #trimeshObj1 = burg.util.o3d_mesh_to_trimesh(mesh1)
    objectType1 = burg.scene.ObjectType(identifier = 'table', mesh = mesh1)
    objectType1.make_urdf_file(directory = '../data/tmp')
    objectInst1 = burg.scene.ObjectInstance(objectType1)

    target_object2 = object_library['bowl']
    mesh2 = target_object2.mesh
    #burg.mesh_processing.check_properties(mesh2)
    objectType2 = burg.scene.ObjectType(identifier = 'bowl', mesh = mesh2, mass = 0.5)
    objectType2.make_urdf_file(directory = '../data/tmp')
    obj_pose1 = np.array([
                [ 1, 0, 0, 0],
                [ 0, 1, 0, 0],
                [ 0, 0, 1, 0.05529334/2],
                [ 0, 0, 0 , 1]])
    objectInst2 = burg.scene.ObjectInstance(objectType2, pose= obj_pose1)

    target_object3 = object_library['softball']
    mesh3 = target_object3.mesh
    #burg.mesh_processing.check_properties(mesh3)
    objectType3 = burg.scene.ObjectType(identifier = 'softball', mesh = mesh3, mass = 0.1)
    objectType3.make_urdf_file(directory = '../data/tmp')
    obj_pose3 = np.array([
                [ 1, 0, 0, 0.3],
                [ 0, 1, 0, 0.3],
                [ 0, 0, 1, 0.09460544/2],
                [ 0, 0, 0 , 1]])
    objectInst3 = burg.scene.ObjectInstance(objectType3, pose = obj_pose3)

    target_object4 = object_library['pear']
    mesh4 = target_object4.mesh
    #burg.mesh_processing.check_properties(mesh4)
    objectType4 = burg.scene.ObjectType(identifier = 'pear', mesh = mesh4, mass = 0.05)
    objectType4.make_urdf_file(directory = '../data/tmp')
    obj_pose4 = np.array([
                [ 1, 0, 0, 0],
                [ 0, 1, 0, 0],
                [ 0, 0, 1, 0.0656779/2],
                [ 0, 0, 0 , 1]])
    objectInst4 = burg.scene.ObjectInstance(objectType4, pose = obj_pose4)

    target_object5 = object_library['foamBrick']
    mesh5 = target_object5.mesh
    objectType5 = burg.scene.ObjectType(identifier = 'foamBrick', mesh = mesh5, mass = 0.05)
    objectType5.make_urdf_file(directory = '../data/tmp')
    obj_pose5 = np.array([
                [ 1, 0, 0, 0.3],
                [ 0, 1, 0, -0.3],
                [ 0, 0, 1, 0.05102145/2],
                [ 0, 0, 0 , 1]])
    objectInst5 = burg.scene.ObjectInstance(objectType5, pose = obj_pose5)

    objects_bg = [objectInst1]
    objects_scene = [objectInst2, objectInst3, objectInst4, objectInst5]
    sc = burg.scene.Scene(objects = objects_scene, bg_objects = objects_bg)


    gripper_model = burg.gripper.Robotiq2F85()
    grasp_pose = np.array([
        [1, 0, 0, 0.3],
        [0, 1, 0, -0.3],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    g = burg.grasp.Grasp()
    g.pose = grasp_pose

    # show grasp for mesh of object type
    burg.visualization.show_grasp_set([objectInst5.object_type.mesh], g, gripper=gripper_model)

    # transform grasp to pose of object instance and show that
    g.transform(objectInst2.pose)
    burg.visualization.show_grasp_set([objectInst5.get_mesh()], g, gripper=gripper_model, with_plane=True)

    # simulation should look like that as well
    sim = burg.sim.SceneGraspSimulator(target_object = objectInst5, gripper= gripper_model, scene=sc, verbose=True)
    #sim = burg.sim.SingleObjectGraspSimulator(target_object = objectInst5, gripper= gripper_model, verbose=True)
    sim.simulate_grasp_set(g)
    sim.dismiss()


if __name__ == "__main__":
    arguments = parse_args()
    cfg = configparser.ConfigParser()
    # read config file and use the section that contains the data paths
    cfg.read(arguments.config_fn)
    training_demo(cfg['General'])
