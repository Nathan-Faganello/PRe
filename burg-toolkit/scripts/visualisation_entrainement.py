import configparser
import open3d as o3d
import numpy as np
import time
import copy

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

target_object_test = object_library['table']
#burg.visualization.show_o3d_point_clouds(target_object_test.mesh, colorize=False)

#generate_dataset.preprocess_shapes(cfg['General'], ['table','bowl', 'softball', 'pear'])

#objects_bg = [objectInst1, planeInst]
#objects_scene = [objectInst2, objectInst3, objectInst4]

print("Testing mesh in open3d ...")
mesh = target_object3.mesh
print(mesh)
print(np.asarray(mesh.vertices))
print(np.asarray(mesh.triangles))
print("")

print("Try to render a mesh with normals (exist: " +
        str(mesh.has_vertex_normals()) + ") and colors (exist: " +
        str(mesh.has_vertex_colors()) + ")")
o3d.visualization.draw_geometries([mesh])
print("A mesh with no normals and no colors does not seem good.")

print("Computing normal and rendering it.")
mesh.compute_vertex_normals()
print(np.asarray(mesh.triangle_normals))
o3d.visualization.draw_geometries([mesh])

print("We make a partial mesh of only the first half triangles.")
mesh1 = copy.deepcopy(mesh)
mesh1.triangles = o3d.utility.Vector3iVector(
    np.asarray(mesh1.triangles)[:len(mesh1.triangles) // 2, :])
mesh1.triangle_normals = o3d.utility.Vector3dVector(
    np.asarray(mesh1.triangle_normals)[:len(mesh1.triangle_normals) //
                                        2, :])
print(mesh1.triangles)
o3d.visualization.draw_geometries([mesh1])

print("Painting the mesh")
mesh1.paint_uniform_color([1, 0.706, 0])
o3d.visualization.draw_geometries([mesh1])


#sc = burg.scene.Scene(objects = objects_scene, bg_objects = objects_bg)

#burg.visualization.show_scene(sc, with_bg_objs = True, add_plane = False)
