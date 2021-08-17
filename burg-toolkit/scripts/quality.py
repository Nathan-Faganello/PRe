import scipy as sc
import numpy as np
import math

from burg_toolkit import sampling 
from scripts import perturbations
import cvxopt as cvx

def epsilon_quality(grasp, param_grasp, force_norm = 50, torque_scaling = 1.0):
    """
    Computes the epsilon metric for a given grasp and associated parameters
    :grasp (Grasp) : grasp pose
    :param_grasps (contact_points(n,3), normals(n,3), approach_vectors(n,3)) : associated parameters
    :force_norm (float) : norm of the force applied along the approach vectors
    :torque_scaling (float) : parameter to adjust torque
    :return [grasp(Grasp), epsilon_quality(float)]
    """

    contact_points = param_grasp[0]
    normals = param_grasp[1]
    approach_vectors = param_grasp[2]

    #1 - Determine the different forces at the contact points : the force along the approach vector is projected on the normal and the normal is separeted along the cone vectors 
    N= len(contact_points)
    num_cone_faces = 8
    contact_forces = np.empty((N, 3, num_cone_faces))
    for i in range(N):
        cone_success, contact_forces_temp = friction_cone(contact_point = contact_points[i], normal = normals[i], num_cone_faces = num_cone_faces)

        if cone_success == False :
            print("the epsilon quality can't be computed")
            return [grasp, 0.0]

        contact_forces[i] = contact_forces_temp


    force_along_normal_norm = 1.0

    #define the wrench_vectors, vectors of the wrench at each contact points
    #the wrench vector is composed of each vector composing the friction cone at each contact point
    wrench_vectors = np.empty((num_cone_faces*N, 6))
    for i in range(N):
        for j in range(num_cone_faces):
            force_along_normal_norm = np.dot((normals[i] / np.linalg.norm(normals[i])), (approach_vectors[i]/np.linalg.norm(approach_vectors[i])))
            """ print("normal : ", (normals[i] / np.linalg.norm(normals[i])))
            print("approach_vector : ", (approach_vectors[i]/np.linalg.norm(approach_vectors[i])))
            print("force_along_normal_norm : ", force_along_normal_norm) 
            """
            wrench_vectors[num_cone_faces*i+j,:3] = force_norm * force_along_normal_norm*contact_forces[i,:,j]
            wrench_vectors[num_cone_faces*i+j,3:] = torque_scaling * torque(wrench_vectors[num_cone_faces*i+j,:3], contact_points[i])
    
    #print(wrench_vectors)

    #compute the convexhull
    #hull = sc.spatial.ConvexHull(wrench_vectors, qhull_options = 'QbB')
    #hull = sc.spatial.ConvexHull(wrench_vectors, qhull_options = 'QJ Pp')
    hull = sc.spatial.ConvexHull(wrench_vectors, qhull_options = 'QJ')
    #hull = sc.spatial.ConvexHull(wrench_vectors)


    #calculate the min distance between the center of the hull and the border of the hull to determine the radius of the biggest ball that can fit in
    radius_min = math.inf
    for v in hull.simplices:
        if np.max(np.array(v)) < wrench_vectors.shape[1]: # because of some occasional odd behavior from pyhull
            facet = wrench_vectors.T[:,v]
            dist, _ = min_norm_vector_in_facet(facet, wrench_regularizer=1)
            if dist < radius_min:
                radius_min = dist


    return radius_min
    

def torque(force, contact_point):

    """
    Given a force and a contact point, it computes the torques at the origin of the coordinate system. We assume that the object mesh has is center of mass at the oririn so the contact points are centered around it
    :force (3,1) : forces along x,y,z
    :contact_point(3,1): coordinates of the contact point (x,y,z)
    :return torques (3,1) along x,y,z
    """
    torques = np.zeros(3)
    moment_arm = contact_point
    torques= np.cross(moment_arm, force)

    return torques



def center_hull(hull):
    """
    For a given hull, find the centroid
    :hull (Scipy.spatial.ConvexHull) : convex hull
    :return centroid (n,1)
    """
    delaun = sc.spatial.Delaunay(hull.points).simplices
    n = delaun.shape[0]
    W = np.zeros(n)
    center = 0
    
    for m in range(n):
        sp = hull.points[T[m, :], :]
        W[m] = sc.spatial.ConvexHull(sp).volume
        center += W[m] * np.mean(sp, axis=0)
    
    return center / np.sum(W)



def friction_cone(contact_point, normal, num_cone_faces=8, friction_coef=0.5):
        """ Computes the friction cone and normal for a contact point.

        :contact_point (3,1) : coordinates of the contact point
        :normal(3,1) : normal at the contact point
        :num_cone_faces(int) number of cone faces to use in discretization
        :friction_coef(float) :coefficient of friction at contact point
        
        Returns

        success : bool
            False when cone can't be computed
        cone_support : :obj:`numpy.ndarray`
            array where each column is a vector on the boundary of the cone
        """

        # get normal and tangents
        t1, t2 = tangents(normal)

        # set up friction cone
        force = normal
        cone_support = np.zeros((3, num_cone_faces))

        # find convex combinations of tangent vectors
        for j in range(num_cone_faces):
            tan_vec = t1 * np.cos(2 * np.pi * (float(j) / num_cone_faces)) + t2 * np.sin(2 * np.pi * (float(j) / num_cone_faces))
            cone_support[:, j] = force + friction_coef * tan_vec

        return True, cone_support
    


def tangents(direction):
    """
    Compute the tangents of a given vector
    :direction (3,1) : reference vector
    :return : v(3,1), w(3,1), the 2 tangent vectors
    """
    #transform in 2D for svd
    direction = direction.reshape((3, 1))
    # get orthogonal plane
    U, _, _ = np.linalg.svd(direction)

    # U[:, 1:] spans the tanget plane at the contact
    x, y = U[:, 0], U[:, 1]

    # make sure t1 and t2 obey right hand rule
    z_hat = np.cross(x, y)
    """ print("z_hat : ", z_hat)
    print("direction : ", direction)
    print("produit scalaire : ", np.dot(z_hat, direction)) """
    if np.dot(z_hat, direction) < 0:
        y = -y
    v = x
    w = y

    return v, w


def probability_force_closure(object, graspset, param_grasps, threshold = 0.001):
    """
    For a given grasp, computes a noisy grasp set and determine the probability of force closure
    :object (objectInst) : grasped object
    :graspset(GraspSet) : list of grasps
    :param_grasps : (contact_points, normals, approach_vectors) for each grasp os the grasp set
    :threshold (float) : threshold used to compare the epsilon metric
    """
    prob = 0

    _, mx = object.mesh.bounding_box()
    torque_scaling = 1.0 / np.median(mx)

    for i, grasp in enumerate(graspset) :
        epsilon = epsilon_quality(object, grasp, param_grasps[i], force = 1, torque_scaling=torque_scaling)
        
        if epsilon[1] > threshold:
            prob += 1
    
    return prob / len(param_grasps)


def min_norm_vector_in_facet(facet, wrench_regularizer=1e-10):
    """ Finds the minimum norm point in the convex hull of a given facet (aka simplex) by solving a QP.
    Parameters
    ----------
    facet : 6xN :obj:`numpy.ndarray`
        vectors forming the facet
    wrench_regularizer : float
        small float to make quadratic program positive semidefinite
    Returns
    -------
    float
        minimum norm of any point in the convex hull of the facet
    Nx1 :obj:`numpy.ndarray`
        vector of coefficients that achieves the minimum
    """
    dim = facet.shape[1] # num vertices in facet

    # create alpha weights for vertices of facet
    G = facet.T.dot(facet)
    grasp_matrix = G + wrench_regularizer * np.eye(G.shape[0])

    # Solve QP to minimize .5 x'Px + q'x subject to Gx <= h, Ax = b
    P = cvx.matrix(2 * grasp_matrix)   # quadratic cost for Euclidean dist
    q = cvx.matrix(np.zeros((dim, 1)))
    G = cvx.matrix(-np.eye(dim))       # greater than zero constraint
    h = cvx.matrix(np.zeros((dim, 1)))
    A = cvx.matrix(np.ones((1, dim)))  # sum constraint to enforce convex
    b = cvx.matrix(np.ones(1))         # combinations of vertices

    sol = cvx.solvers.qp(P, q, G, h, A, b)
    v = np.array(sol['x'])
    min_norm = np.sqrt(sol['primal objective'])

    return abs(min_norm), v
