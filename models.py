import math
import numpy as np


def unit_vector(vector):
    """ Returns the unit vector of the vector."""
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'"""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def path_decompiler(lines):
    """
    decompiles a path from its geometry configuration into a pure list of tuples
    :param  lines:      list in geometric form according to osmnx 'geometry' feature
    :return new_path:   list of tuples
    """
    path = []
    for line in lines:
        for point in line:
            path.append(point)

    # the path must be cleaned of twin nodes for car dynamics
    # these are nodes which overlap (two nodes laying on top of each other on the same point)
    # OpenStreetMap has this issue
    clean_path = []
    for i in range(len(path)):
        if (i < len(path) - 1) and (path[i] == path[i + 1]):
            continue
        else:
            clean_path.append(path[i])

    return path


def upcoming_vectors(view):
    """
    determines the vectors between the nodes in a view
    :param     view: list:  list of n upcoming nodes
    :return vectors: list:  list of (n-1) vectors pointing between the nodes along the path of travel
    """
    vectors = []
    for i in range(len(view)):
        if i < len(view) - 1:
            vectors.append(np.array([view[i + 1][0] - view[i][0], view[i + 1][1] - view[i][1]]) /
                           math.sqrt(np.dot(view[i], view[i + 1])))
    return vectors


def get_angles(view):
    """
    determines the angles between the upcoming vectors
    :param   view: list: list of coordinate points of next five nodes in path
    :return  angles: list: list of the next angles of road curvature
    """
    vectors = upcoming_vectors(view)
    angles = []
    for i in range(len(vectors)):
        if i < len(vectors) - 1:
            angles.append(angle_between(vectors[i], vectors[i + 1]))

    return angles


def get_distances(view):
    """
    determines the upcoming distances (lengths of upcoming_vectors)
    :param        view: list: list of coordinate points of next five nodes in path
    :return: distances: list: list of the next distances between upcoming nodes on the road
    """
    vectors = upcoming_vectors(view)
    distances = [np.dot(vector, vector) for vector in vectors]
    return distances



