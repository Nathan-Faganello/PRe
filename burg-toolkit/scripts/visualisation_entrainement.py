import configparser
import open3d as o3d
import numpy as np

import burg_toolkit as burg
from scripts import generate_dataset

cfg_fn = '../config/config.cfg'
print('using config file in:', cfg_fn)

cfg = configparser.ConfigParser()
cfg.read(cfg_fn)
reader = burg.io.BaseviMatlabScenesReader(cfg['General'])

# load object library
print('read object library')
object_library, index2name = reader.read_object_library()
object_library.yell()


target_object1 = object_library['table']
mesh1 = target_object1.mesh
#burg.mesh_processing.check_properties(mesh1)
#trimeshObj1 = burg.util.o3d_mesh_to_trimesh(mesh1)
objectType1 = burg.scene.ObjectType(identifier = 'table', mesh = mesh1)
objectInst1 = burg.scene.ObjectInstance(objectType1)

target_object2 = object_library['bowl']
mesh2 = target_object2.mesh
#burg.mesh_processing.check_properties(mesh2)
objectType2 = burg.scene.ObjectType(identifier = 'bowl', mesh = mesh2, mass = 0.5)
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
obj_pose4 = np.array([
            [ 1, 0, 0, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 1, 0.0656779/2],
            [ 0, 0, 0 , 1]])
objectInst4 = burg.scene.ObjectInstance(objectType4, pose = obj_pose4)

plane = burg.visualization.create_plane(1.5,1,0.01)
planeType = burg.scene.ObjectType(mesh = plane)
planeInst = burg.scene.ObjectInstance(planeType)


#burg.visualization.show_o3d_point_clouds(mesh, colorize=False)

#generate_dataset.preprocess_shapes(cfg['General'], ['table','bowl', 'softball', 'pear'])

objects_bg = [objectInst1, planeInst]
objects_scene = [objectInst2, objectInst3, objectInst4]

sc = burg.scene.Scene(objects = objects_scene, bg_objects = objects_bg)

burg.visualization.show_scene(sc, with_bg_objs = True, add_plane = False)
