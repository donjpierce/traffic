"""
Description of module...
"""
import math
import models
import navigation as nav
import numpy as np


# fill the initial state with N cars
speed_limit = 200
stop_distance = 5
free_distance = 30

TEMP_dest_node = 53028190   # Piedmont destination
# TEMP_dest_node = 1989931095  # Manhattan destination
# TEMP_dest_node = 65307322  # San Francisco destination


def update_path(car):
    """

    :param   car: dict
    :return path: origin path if stored node
    """
    if len(car['path']) <= 1:
        return car['path']

    obstacles = nav.FrontView(car)
    if obstacles.crossed_node_event():
        return car['path'][1:]
    else:
        return car['path']


def update_velocity(car):
    """
    updates velocity according to the forward Euler method

    :param       car: dict
    :return velocity: list
    """
    if len(car['path']) < 1:
        velocity = np.array([0., 0.])
        return velocity
    next_node = car['path'][0]
    position = np.array(car['position'])
    velocity_direction = models.unit_vector(next_node - position)
    velocity = velocity_direction * speed_limit * update_speed_factor(car)
    return velocity


def update_speed_factor(car):
    """
    handles logic for updating speed according to road curvature and car obstacles

    :param            car: dict
    :return: speed_factor: double
    """
    obstacles = nav.FrontView(car)
    angles = obstacles.angles
    distance_to_node = obstacles.distance_to_node()
    distance_to_car = car['distance-to-car']
    car_factor = car_obstacle_factor(distance_to_car)  # for later use with car obstacles
    speed_factor = road_curvature_factor(angles, distance_to_node)
    return speed_factor


def road_curvature_factor(angles, d):
    """
    calculates the speed factor (between 0 and 1) for road curvature

    Parameters
    __________
    :param        angles: double:  angles of road curvature ahead
    :param             d: double:  distance from car to next node

    Returns
    _______
    :return speed_factor: double:  factor by which to diminish speed
    """
    if len(angles) < 1:
        # if it's the end of the route, treat the last node like a hard-stop intersection
        theta = math.pi/2
    else:
        theta = angles[0]

    if theta == 0:
        curvature_factor = 1
    else:
        if np.isclose(stop_distance, d, rtol=1.0e-3):
            curvature_factor = 0.01
        else:
            if (stop_distance < d) and (d < free_distance):
                curvature_factor = math.log(d / (stop_distance * 2 * theta / math.pi)) / \
                                   math.log(free_distance / (stop_distance * 2 * theta / math.pi))
            else:
                curvature_factor = 1
    return curvature_factor


def car_obstacle_factor(d):
    """
    function to update speed for a car in the front_view

    :param      d: double:   distance to car in front_view
    :return speed: double:  new speed
    """

    if (stop_distance < d) and (d < free_distance):
        obstacle_factor = math.log(d / stop_distance) / math.log(free_distance / stop_distance)
    else:
        if d < stop_distance:
            obstacle_factor = 0
        else:
            obstacle_factor = 1
    return obstacle_factor


def init_random_node_start_location(n):
    """
    initializes n cars at n random nodes

    :param      n: int
    :return state: dict
    """
    nodes = nav.find_nodes(n)

    cars = []

    for i in range(n):
        start_node = nodes[i]
        position = nav.get_position_of_node(start_node)
        cars.append(
            {'position': position,
             'velocity': np.array([0, 0]),
             'acceleration': np.array([0, 0]),
             'front-view': {'distance-to-car': 0, 'distance-to-node': 0},
             'origin': start_node,
             'destination': TEMP_dest_node
             }
        )
        cars[i]['path'] = np.array(nav.get_init_path(cars[i]))
        cars[i]['front-view']['distance-to-node'] = nav.FrontView(cars[i]).distances[0]

    return cars


def init_culdesac_start_location(n):
    """
    initializes N cars into N culdesacs

    Parameters
    __________
    :param     n:   int

    Returns
    _______
    :return cars:   array:  [dict, ...]
    """
    culdesacs = nav.find_culdesacs()

    if n > len(culdesacs):
        raise ValueError('Number of cars greater than culdesacs to place them. '
                         'Choose a number less than {}'.format(len(culdesacs)))

    cars = []

    for i in range(n):
        start_node = culdesacs[i]
        position = nav.get_position_of_node(start_node)
        cars.append(
            {'position': position,
             'velocity': np.array([0, 0]),
             'acceleration': np.array([0, 0]),
             'front-view': {'distance-to-car': 0, 'distance-to-node': 0},
             'origin': start_node,
             'destination': TEMP_dest_node
             }
        )
        cars[i]['path'] = np.array(nav.get_init_path(cars[i]))
        cars[i]['front-view']['distance-to-node'] = nav.FrontView(cars[i]).distances[0]

    return cars
