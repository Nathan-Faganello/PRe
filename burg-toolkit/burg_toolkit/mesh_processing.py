import numpy as np
import trimesh
import open3d as o3d
import random as rd
import copy

from burg_toolkit import util


def check_properties(mesh):
    """
    Utility function to check properties of a mesh. Will be printed to standard output.

    :param mesh: open3d.geometry.TriangleMesh
    """
    has_triangle_normals = mesh.has_triangle_normals()
    has_vertex_normals = mesh.has_vertex_normals()
    has_texture = mesh.has_textures()
    edge_manifold = mesh.is_edge_manifold(allow_boundary_edges=True)
    edge_manifold_boundary = mesh.is_edge_manifold(allow_boundary_edges=False)
    vertex_manifold = mesh.is_vertex_manifold()
    self_intersecting = mesh.is_self_intersecting()
    watertight = mesh.is_watertight()
    orientable = mesh.is_orientable()
    _trimesh = util.o3d_mesh_to_trimesh(mesh)
    convex = trimesh.convex.is_convex(_trimesh)

    print(f"  no vertices:            {len(mesh.vertices)}")
    print(f"  no triangles:           {len(mesh.triangles)}")
    print(f"  dims (x, y, z):         {dimensions(mesh)}")
    print(f"  has triangle normals:   {has_triangle_normals}")
    print(f"  has vertex normals:     {has_vertex_normals}")
    print(f"  has textures:           {has_texture}")
    print(f"  edge_manifold:          {edge_manifold}")
    print(f"  edge_manifold_boundary: {edge_manifold_boundary}")
    print(f"  vertex_manifold:        {vertex_manifold}")
    print(f"  self_intersecting:      {self_intersecting}")
    print(f"  watertight:             {watertight}")
    print(f"  orientable:             {orientable}")
    print(f"  convex:                 {convex}")
    print(f"  components:             {_trimesh.body_count}")


def dimensions(mesh):
    """
    Returns the extent of the mesh in [x, y, z] directions, i.e. of the axis aligned bbox.

    :param mesh: open3d.geometry.TriangleMesh

    :return: (3) np array
    """
    return mesh.get_max_bound().flatten() - mesh.get_min_bound().flatten()


def poisson_disk_sampling(mesh, radius=0.003, n_points=None, with_normals=True, init_factor=5):
    """
    Performs poisson disk sampling.
    Per default it uses the radius, but if `n_points` is given it just samples as many points.
    Ideally, we fit circles with a certain radius on the surface area of the mesh. The center points
    will then form the point cloud. In practice, we just randomly sample a certain number of points (depending
    on the surface area of the mesh, the radius and init_factor), then we eliminate points which do not fit well.

    :param mesh: open3d.geometry.TriangleMesh
    :param radius: the smaller the radius, the higher the point density will be
    :param init_factor: we will initially sample `init_factor` times more points than will be needed, if better
                        accuracy is desired this can be increased, if performance is important this should be decreased
    :param with_normals: whether or not the points shall have normals (default is True)
    :param n_points: int, if given, it will simply sample as many points (which are approx evenly distributed)

    :return: open3d.geometry.PointCloud object
    """
    if n_points is None:
        # we assume the surface area to be square and use a square packing of circles
        # the number n_s of circles along the side-length s can then be estimated with
        s = np.sqrt(mesh.get_surface_area())
        n_s = (s + 2*radius) / (2*radius)
        n_points = int(n_s**2)
        print(f'going for {n_points} points')

    pc = mesh.sample_points_poisson_disk(
        number_of_points=n_points,
        init_factor=init_factor,
        use_triangle_normal=with_normals
    )

    return pc


def compute_mesh_inertia(o3d_mesh, mass):
    """
    Tries to compute the inertia of the given mesh. Will only work if mesh is watertight.

    :param o3d_mesh: open3d.geometry.TriangleMesh
    :param mass: mass of object in kg

    :return: moment of inertia matrix, (3, 3) float ndarray, origin center of mass (3) float ndarray
    """
    mesh = util.o3d_mesh_to_trimesh(o3d_mesh)
    if not mesh.is_watertight:
        raise ValueError('cannot compute inertia, mesh is not watertight.')

    # trimesh meshes are density-based, so let's set density based on given mass and mesh volume
    mesh.density = mass / mesh.volume
    return mesh.moment_inertia, mesh.center_mass



def select_stable_pose(mesh):
    # among all the possible poses for the mesh, it returns the most probable
    # mesh = the mesh for which we need to have the stable pose
    tri = util.o3d_mesh_to_trimesh(mesh)
    stable_poses, proba = trimesh.poses.compute_stable_poses(tri)
    N = len(stable_poses)
    proba_max = proba[0]
    index_max = 0
    for index in range (1,N):
        if (proba[index]>proba_max):
            proba_max = proba[index]
            index_max = index 
    return stable_poses[index_max]

"""
def select_random_stable_pose(mesh):
    #select a pose among all the possible poses randomly
    tri = util.o3d_mesh_to_trimesh(mesh)
    stable_poses, proba = trimesh.poses.compute_stable_poses(tri)
    N = len(stable_poses)
    index = rd.randint(0,N-1)
    return stable_poses[index]
"""

def select_random_stable_pose(mesh):
    """
    Select a pose among stable poses
    """
    tri = util.o3d_mesh_to_trimesh(mesh)
    stable_poses, proba = trimesh.poses.compute_stable_poses(tri)

    stable_poses, proba = stable_poses[:32], proba[:32]
    proba = [x * 100 for x in proba]
    proba = [x / sum(proba) for x in proba]
    index_list = [k for k in range(len(stable_poses))]

    index_choosen = np.random.choice(index_list, p = proba)

    stable_pose = stable_poses[index_choosen]

    return stable_pose



""" def are_colliding_2(object1, object2):

    Check if 2 object meshes are in collision using python-fcl
    :object1 (objectInstance) : first object we want to check
    :object2 (objectInstance) : second object we want to check
    :return (bool) : True if they are colliding, False if not


    collision_manager = trimesh.collision.CollisionManager()
    collision_manager.addObject(name = object1.identifier, mesh = object1.mesh, transform = object1.pose)
    collision_manager.addObject(name = object2.identifier, mesh = object2.mesh, transform = object2.pose)

    collision = collision_manager.in_collision_internal(return_names = False, return_data = False)

    return collision

def are_colliding_list(objects, obj1):

    Check if several object meshes are in collision with another object using python-fcl
    :objects (list of objectInstance) : objects we want to check
    :return (bool) : True if they are colliding, False if not

    collision_manager = trimesh.collision.CollisionManager()
    collision_manager.addObject(name = obj1.identifier, mesh = obj1.mesh, transform = obj1.pose)

    for obj in objects :
        collision_manager.addObject(name = obj.identifier, mesh = obj.mesh, transform = obj.pose)

    collision = collision_manager.in_collision_internal(return_names = False, return_data = False)

    return collision

def check_collision(object1, object2):

    Check if 2 object meshes are in collision using AxisAlignedBoundingBoxes
    :object1 (objectInstance) : first object we want to check
    :object2 (objectInstance) : second object we want to check
    :return (bool) : True if they are colliding, False if not

    pc1 = poisson_disk_sampling(object1.object_type.mesh, init_factor=7)
    pc2 = poisson_disk_sampling(object2.object_type.mesh, init_factor=7)

    aabb1 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pc1.points))
    aabb2 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pc2.points))

    pose1 = object1.pose
    pose2 = object2.pose

    #bounds = [x, y, z]
    max_bounds1 = aabb1.get_max_bound()
    max_bounds2 = aabb2.get_max_bound()
    min_bounds1 = aabb1.get_min_bound()
    min_bounds2 = aabb2.get_min_bound()

    #the meshes are not translated in space so we need to adjust coordinates based on the pose of the object in the scene
    for axis in range(3):
        max_bounds1[axis] += pose1[axis][2]
        max_bounds1[axis] += pose2[axis][2]
        min_bounds1[axis] += pose1[axis][2]
        min_bounds2[axis] += pose2[axis][2]
    

    #We can now check if it's colliding

    colliding_x = True
    colliding_y = True
    colliding_z = True

    # x_axis
    if min_bounds2[0] > max_bounds1[0] or max_bounds2[0] < min_bounds1[0]:
        colliding_x = False
    
    # y_axis
    if min_bounds2[1] > max_bounds1[1] or max_bounds2[1] < min_bounds1[1]:
        colliding_y = False
    
    #z_axis
    if min_bounds2[2] > max_bounds1[2] or max_bounds2[2] < min_bounds1[2]:
        colliding_z = False

    return colliding_x and colliding_z and colliding_y

"""

def check_collision(object1, object2):

    """
    
    Check if 2 object meshes are in collision using AxisAlignedBoundingBoxes
    :object1 (objectInstance) : first object we want to check
    :object2 (objectInstance) : second object we want to check
    :return (bool) : True if they are colliding, False if not
    
    """

    # 1 - get the meshes
    mesh1 = copy.copy(object1.object_type.mesh)
    mesh2 = copy.copy(object2.object_type.mesh)

    # 2 - transform the meshes
    transl_vect1 = [object1.pose[0,3], object1.pose[1,3], object1.pose[2,3]]
    transl_vect2 = [object2.pose[0,3], object2.pose[1,3], object2.pose[2,3]]

    mesh1 = mesh1.translate(transl_vect1)
    mesh2 = mesh2.translate(transl_vect2)

    # 3 - check collisions
    return mesh1.is_intersecting(mesh2)


     
