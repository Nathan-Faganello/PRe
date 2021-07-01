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
objectInst1 = burg.scene.ObjectInstance(target_object1)

target_object2 = object_library['bowl']
objectType2 = burg.scene.ObjectType(identifier = 'bowl', mass = 0.5)
obj_pose1 = np.array([
            [ 1, 0, 0, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 1, 0.05529334/2],
            [ 0, 0, 0 , 1]])
objectInst2 = burg.scene.ObjectInstance(objectType2, pose= obj_pose1)

target_object3 = object_library['softball']
objectType3 = burg.scene.ObjectType(identifier = 'softball', mass = 0.1)
obj_pose3 = np.array([
            [ 1, 0, 0, 0.3],
            [ 0, 1, 0, 0.3],
            [ 0, 0, 1, 0.09460544/2],
            [ 0, 0, 0 , 1]])
objectInst3 = burg.scene.ObjectInstance(objectType3, pose = obj_pose3)

target_object4 = object_library['pear']
objectType4 = burg.scene.ObjectType(identifier = 'pear', mass = 0.05)
obj_pose4 = np.array([
            [ 1, 0, 0, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 1, 0.0656779/2],
            [ 0, 0, 0 , 1]])
objectInst4 = burg.scene.ObjectInstance(objectType4, pose = obj_pose4)

#burg.visualization.show_o3d_point_clouds(mesh, colorize=False)

#generate_dataset.preprocess_shapes(cfg['General'], ['table','bowl', 'softball', 'pear'])

objects_bg = [objectInst1]
objects_scene = [objectInst2, objectInst3, objectInst4]

sc = burg.scene.Scene(objects = objects_scene, bg_objects = objects_bg)

burg.visualization.show_scene(sc, with_bg_objs = True, add_plane = True)
