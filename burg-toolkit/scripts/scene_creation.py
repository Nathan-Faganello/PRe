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

"""
Following function uses python-fcl
def scene_creation_random(data_conf):

    #Without python-fcl, I don't know how to check collisions between meshes. It makes this function unusable

    reader = burg.io.BaseviMatlabScenesReader(data_conf)

    print('read object library')
    object_library, index2name = reader.read_object_library()
    object_library.yell()
    
    print('generating urdf files for object library')
    object_library.generate_urdf_files('../data/tmp')

    # 1 - load objects and place them in a stable / non-colliding pose

    #We limit the position of object to a restricted area

    x_lim = 15t
    y_lim = 15

    objects = []
    buffer = input("Select an object in the previous list (type end to quit the object selection) : ")
    while buffer != "end" :
        if buffer in [key for key in object_library.data.keys()] :
            target_object = object_library[buffer]
            mesh = target_object.mesh
            stable_pose = burg.mesh_processing.select_stable_pose(mesh)

            placed = False
            tries = 0
            while placed == False and tries < 10 :
                x_transl = rd.uniform(-x_lim, x_lim)
                y_transl = rd.uniform(-y_lim, y_lim)
                transl_tf = burg.util.tf_translation(x_transl, "x") @ burg.util.tf_translation(y_transl, "y")
                objectInst = burg.scene.ObjectInstance(target_object, pose = stable_pose @ transl_tf)
                placed =  not burg.mesh_processing.are_colliding_list(objects, objectInst)
                tries += 1

            if placed == True:
                objects += [objectInst]
                print("The object has been added to the scene successfully")
            else : 
                print("The object can't be placed in the scene")
        else :
            print("the object is not in the list")

        buffer = input("Select an object in the previous list (type end to quit the object selection) : ")

    # 2 - create and visualize the scene

    sc = burg.scene.Scene(objects = objects_scene)
    burg.visualization.show_scene(sc)
"""
"""
following function creates a sim for each collision detection
def scene_creation_random(data_conf):
    reader = burg.io.BaseviMatlabScenesReader(data_conf)

    print('read object library')
    object_library, index2name = reader.read_object_library()
    object_library.yell()
    
    print('generating urdf files for object library')
    object_library.generate_urdf_files('../data/tmp')

    gripper_model = burg.gripper.Robotiq2F85()
    g = burg.grasp.Grasp()
    g.pose = np.eye(4)

    # 1 - load objects and place them in a stable / non-colliding pose
    #We limit the position of object to a restricted area
    x_lim = 0.5
    y_lim = 0.5

    buffer = input("Select an object in the previous list (type end to quit the object selection) : ")
    obj_names = []
    while buffer != "end":
        if buffer in [key for key in object_library.data.keys()] :
            obj_names += [buffer]
            print(buffer," is added")
        else :
            print("the object is not in the list")
        
        buffer = input("Select an object in the previous list (type end to quit the object selection) : ")
    
    print("You have selected all the objects. They are going to be placed randomly")

    finalObjects = []
    tempObjects = []
    for obj_name in obj_names:
        target_object = object_library[obj_name]
        mesh = target_object.mesh
        stable_pose = burg.mesh_processing.select_stable_pose(mesh)

        placed = False
        tries = 0
        while placed == False and tries < 10 :
            x_transl = rd.uniform(-x_lim, x_lim)
            y_transl = rd.uniform(-y_lim, y_lim)
            transl_tf = burg.util.tf_translation(x_transl, "x") @ burg.util.tf_translation(y_transl, "y")
            objectInst = burg.scene.ObjectInstance(target_object, pose = stable_pose @ transl_tf)
            tempObjects += [objectInst]

            tempScene = burg.scene.Scene(objects = tempObjects)
            tempSim = burg.sim.SceneGraspSimulator(target_object = objectInst, gripper = g, scene = tempScene)
            tempSim._prepare()
            colliding = False
            for index1, obj1 in enumerate(tempSim._objects_ids) :
                for index2, obj2 in enumerate(tempSim._objects_ids):
                    if index2>index1:
                        if tempSim._are_in_collision(tempSim._objects_ids[obj1], tempSim._objects_ids[obj2]):
                            colliding = True
            if colliding == True :
                tempObjects.pop()
            else :
                placed = True
                finalObjects += [objectInst]
            tries += 1

    # 2 - create and visualize the scene

    sc = burg.scene.Scene(objects = finalObjects)
    burg.visualization.show_scene(sc, add_plane = True)

    return sc
"""

def scene_creation(data_conf):

    reader = burg.io.BaseviMatlabScenesReader(data_conf)

    print('read object library')
    object_library, index2name = reader.read_object_library()
    object_library.yell()
    
    print('generating urdf files for object library')
    object_library.generate_urdf_files('../data/tmp')

    gripper_model = burg.gripper.Robotiq2F85()
    g = burg.grasp.Grasp()
    g.pose = np.eye(4)

    # 1 - load objects and place them in a stable / non-colliding pose
    """
    We limit the position of object to a restricted area
    """
    x_lim = 0.5
    y_lim = 0.5
    str_lim_x = "x position (between -" + str(x_lim) + " and +"+ str(x_lim) + ") : "
    str_lim_y = "y position (between -" + str(y_lim) + " and +"+ str(y_lim) + ") : "

    finalObjects = []
    tempObjects = []
    buffer = input("Select an object in the previous list (type end to quit the object selection) : ")
    while buffer != "end" :
        if buffer in [key for key in object_library.data.keys()] :
            target_object = object_library[buffer]
            mesh = target_object.mesh
            stable_pose = burg.mesh_processing.select_stable_pose(mesh)

            x_transl = float(input(str_lim_x))
            y_transl = float(input(str_lim_y))

            transl_tf = burg.util.tf_translation(x_transl, "x") @ burg.util.tf_translation(y_transl, "y")
            objectInst = burg.scene.ObjectInstance(target_object, pose = stable_pose @ transl_tf)

            colliding = False
            for obj in finalObjects:
                if burg.mesh_processing.check_collision(obj, objectInst):
                    colliding = True
            if colliding == False :
                finalObjects += [objectInst]
            else : 
                print("The object can't be placed, please choose other coordinates")    
        else :
            print("the object is not in the list")

        buffer = input("Select an object in the previous list (type end to quit the object selection) : ")

    # 2 - create and visualize the scene

    sc = burg.scene.Scene(objects = finalObjects)
    burg.visualization.show_scene(sc, add_plane = True)
    return scene


def scene_creation_random(data_conf):
    reader = burg.io.BaseviMatlabScenesReader(data_conf)

    print('read object library')
    object_library, index2name = reader.read_object_library()
    object_library.yell()
    
    print('generating urdf files for object library')
    object_library.generate_urdf_files('../data/tmp')

    gripper_model = burg.gripper.Robotiq2F85()
    g = burg.grasp.Grasp()
    g.pose = np.eye(4)

    # 1 - load objects and place them in a stable / non-colliding pose
    """
    #We limit the position of object to a restricted area
    """
    x_lim = 0.5
    y_lim = 0.5

    buffer = input("Select an object in the previous list (type end to quit the object selection) : ")
    obj_names = []
    while buffer != "end":
        if buffer in [key for key in object_library.data.keys()] :
            obj_names += [buffer]
            print(buffer," is added")
        else :
            print("the object is not in the list")
        
        buffer = input("Select an object in the previous list (type end to quit the object selection) : ")
    
    print("You have selected all the objects. They are going to be placed randomly")

    finalObjects = []
    tries_lim = 10
    tries = 0
    while len(obj_names) != 0 and tries < 10 :
        for index, obj_name in enumerate(obj_names):
            target_object = object_library[obj_name]
            mesh = target_object.mesh
            stable_pose = burg.mesh_processing.select_random_stable_pose(mesh)

            placed = False
            while placed == False and tries < tries_lim :
                x_transl = rd.uniform(-x_lim, x_lim)
                y_transl = rd.uniform(-y_lim, y_lim)
                transl_tf = burg.util.tf_translation(x_transl, "x") @ burg.util.tf_translation(y_transl, "y")
                objectInst = burg.scene.ObjectInstance(target_object, pose = transl_tf @ stable_pose)
                colliding = False
                for obj in finalObjects:
                    if burg.mesh_processing.check_collision(obj, objectInst):
                        colliding = True

                if colliding == False :
                    placed = True
                    finalObjects += [objectInst]
                    obj_names.pop(index)
        tries += 1 

    if len(obj_names) != 0 :
        print(obj_names, " can't be placed")

    # 2 - create and visualize the scene

    sc = burg.scene.Scene(objects = finalObjects)
    burg.visualization.show_scene(sc, add_plane = True)

    return sc


if __name__ == "__main__":
    arguments = parse_args()
    cfg = configparser.ConfigParser()
    # read config file and use the section that contains the data paths
    cfg.read(arguments.config_fn)
    #training_demo(cfg['General'])
    sc = scene_creation_random(cfg['General'])
    