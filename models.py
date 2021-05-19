import math
import numpy as np
import pandas as pd
import random


def determine_bins(axis, df):
    """
    For a given dataframe of objects on the map with an axis, return a Series of xbins and ybins for those objects

    :param axis:
    :param  df:

    :return xbins, ybins:
    """
    xbins, ybins = np.arange(axis[0], axis[1], 200), np.arange(axis[2], axis[3], 200)
    x_indices, y_indices = np.digitize(df['x'], xbins), np.digitize(df['y'], ybins)
    xbins, ybins = pd.Series(x_indices), pd.Series(y_indices)
    return xbins, ybins


def initial_light_colors(n):
    """

    :return:
    """
    init_colors = [random.choice(['red', 'green']) for c in range(n)]
    return init_colors


def determine_traffic_light_timer(degree):
    """
    For now, wait times are determined by taking a random fraction of the degree

    :return random_wait: double: wait time in units of dt
    """
    random_wait = round(random.random() * degree, 2)
    return random_wait


def weigh_factors(car_factor, curvature_factor, distance_to_car, distance_to_node, free_distance):
    """
    weights factors in quadrant I of a unit circle

    Parameters
    __________
    :param       car_factor:    double
    :param curvature_factor:    double
    :param  distance_to_car:    double
    :param distance_to_node:    double
    :param    free_distance:    int or double

    Returns
    _______
    :return factor: double
    """
    # normalize distances
    distance_to_car = distance_to_car / free_distance
    distance_to_node = distance_to_node / free_distance

    # superpose the two distances on a unit circle depending on their relative magnitude
    factor = car_factor * math.cos(distance_to_car) + curvature_factor * math.sin(distance_to_node)

    return factor


def magnitude(vector):
    """ Returns the magnitude of a vector """
    return np.linalg.norm(vector)


def unit_vector(vector):
    """ Returns the unit vector of the vector """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'"""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    # normalize angle to pi/2
    if angle > math.pi/2:
        angle = angle - math.pi/2
    return angle


def determine_anti_parallel_vectors(v1, v2):
    """ Returns True if two vectors are close to parallel """
    v1, v2 = unit_vector(v1), unit_vector(v2)
    angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
    if np.isclose(math.pi, angle, atol=0.1):
        return True
    else:
        return False


def clean_list(alist):
    """
    Simply removes duplicates from a list

    :param   alist: list
    :return clean: list
    """
    clean = []
    for i in range(len(alist)):
        if (i < len(alist) - 1) and (alist[i] != alist[i + 1]):
            clean.append(alist[i])

    clean.append(alist[-1])
    return clean


def path_decompiler(lines):
    """
    Decompiles a path from its geometry configuration into a pure list of tuples

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
        if (i < len(path) - 1) and (path[i] != path[i + 1]):
            clean_path.append(path[i])
    clean_path.append(path[-1])
    return clean_path


def new_route_decompiler(new_path):
    """
    Decompiles a new_path from nav.build_new_route

    :param      new_path:
    :return:  clean_path:
    """
    clean_path = []
    for i in range(len(new_path)):
        if (i < len(new_path) - 1) and (new_path[i] != new_path[i + 1]):
            clean_path.append(new_path[i])
    clean_path.append(new_path[-1])
    return clean_path


def upcoming_linspace(frontview):
    """
    this function returns a 2D linspace between the next two nodes in the view

    :param     frontview: object: FrontView object
    :return        space:  tuple: of np.arrays
    """
    # TODO: don't make linspace so large. nx and ny should not be so large. it's unnecessary
    next_node = frontview.upcoming_node_position()

    x_distance_between = abs(next_node[0] - frontview.car['x'])
    y_distance_between = abs(next_node[1] - frontview.car['y'])

    nx, ny = int(x_distance_between), int(y_distance_between)
    x = np.linspace(frontview.car['x'], next_node[0], nx)
    y = np.linspace(frontview.car['y'], next_node[1], ny)
    return x, y


def upcoming_vectors(view):
    """
    determines the vectors between the nodes in a view

    :param     view:     tuple:  tuple (x,y) of lists representing n upcoming noce positions
    :return vectors:      list:  list of (n-1) vectors pointing between the nodes along the path of travel
    """
    vectors = []
    for i in range(len(view)):
        if i < len(view) - 1:
            vectors.append(np.array([
                view[i + 1][0] - view[i][0], view[i + 1][1] - view[i][1]]
            ) / math.sqrt(np.dot(view[i], view[i + 1])))
    return np.array(vectors)


def get_angles(view):
    """
    determines the angles between the upcoming vectors

    :param        view:   list: list of coordinate points of next five nodes in path
    :return  angles[0]: double: the next angle of the road curvature
    """
    if view and len(view) >= 2:
        vectors = upcoming_vectors(view)
        angles = []
        for i in range(len(vectors)):
            if i < len(vectors) - 1:
                angles.append(angle_between(vectors[i], vectors[i + 1]))
        if angles:
            return angles[0]
        else:
            return False
    else:
        return False


def make_table(dictionary):
    """
    Simply creates a Pandas table

    :param   dictionary: dictionary
    :return          df: DataFrame
    """
    return pd.DataFrame(dictionary)
