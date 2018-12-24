import math
import numpy as np
import pandas as pd
import random


def radixsort(n_nodes, df):
    """
    uses the dataframe and information about the graph to sort the cars and traffic lights by position

    Parameters
    __________
    :param   n_nodes: number of nodes in the graph
    :param        df: pandas dataframe

    Returns
    _______
    :return dataframe: sorted pandas dataframe
    """


    return 0


def initial_light_colors(n):
    """

    :return:
    """
    init_colors = [random.choice(['red', 'green']) for c in range(n)]
    return init_colors


def determine_traffic_light_timer():
    """
    For now, wait times are determined by taking a random fraction of the degree

    :return random_wait: double: wait time in units of dt
    """
    random_wait = round(random.random() * 5, 2)
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


def determine_parralel_vectors(v1, v2):
    """ Returns True if two vectors are close to parallel """
    v1, v2 = unit_vector(v1), unit_vector(v2)
    angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
    if np.isclose([0., math.pi], angle, rtol=1.0e-3).any():
        return True
    else:
        return False


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
        if (i < len(path) - 1) and (path[i] == path[i + 1]):
            continue
        else:
            clean_path.append(path[i])

    return clean_path


def immediate_linspace(view, position):
    """
    this function returns a 2D linspace between the car's immediate position and the next node in the view

    :param     view:    list:   list of n upcoming nodes
    :param position:    list:   coordinate position of car
    :return:  space:   tuple:
    """
    next_node = view[0]

    x_distance_between = abs(next_node[0] - position[0])
    y_distance_between = abs(next_node[1] - position[1])

    nx, ny = (x_distance_between, y_distance_between)
    x = np.linspace(position[0], next_node[0], nx)
    y = np.linspace(position[1], next_node[1], ny)
    space = (x, y)
    return space


def upcoming_linspace(view, position):
    """
    this function returns a 2D linspace between the next two nodes in the view

    :param     view:  list: list of n upcoming nodes
    :param position:  list: coordinate position of car
    :return   space: tuple: of np.arrays
    """
    if len(view) < 2:
        # the car has reached the end of the route
        nearest_node = position
        next_node = view[0]
    else:
        nearest_node = view[0]
        next_node = view[1]

    x_distance_between = abs(next_node[0] - nearest_node[0])
    y_distance_between = abs(next_node[1] - nearest_node[1])

    nx, ny = (x_distance_between, y_distance_between)
    x = np.linspace(nearest_node[0], next_node[0], nx)
    y = np.linspace(nearest_node[1], next_node[1], ny)
    space = (x, y)
    return space


def upcoming_vectors(view):
    """
    determines the vectors between the nodes in a view

    :param     view:     tuple:  tuple (x,y) of lists representing n upcoming noce positions
    :return vectors:      list:  list of (n-1) vectors pointing between the nodes along the path of travel
    """
    position_view = []
    for point in view:
        position_view.append(point)

    vectors = []
    for i in range(len(position_view)):
        if i < len(position_view) - 1:
            vectors.append(np.array([
                position_view[i + 1][0] - position_view[i][0], position_view[i + 1][1] - position_view[i][1]]
            ) / math.sqrt(np.dot(position_view[i], position_view[i + 1])))
    return np.array(vectors)


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

