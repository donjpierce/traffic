"""
Description of module...
"""
import math
import models
import navigation as nav
import numpy as np


# fill the initial state with N cars
speed_limit = 200
stop_distance = 10
free_distance = 30
default_acceleration = 5

TEMP_dest_node = 53028190   # Piedmont destination
# TEMP_dest_node = 1989931095  # Manhattan destination
# TEMP_dest_node = 65307322  # San Francisco destination


def update_path(car):
    """
    This function shortens the stored path of a car after determining if the car crossed the next node in the path

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

    if np.isclose(0, velocity, atol=0.1).all() and accelerate(car):
        velocity += default_acceleration

    return velocity


def accelerate(car):
    """
    determines if there is a car ahead. If there is, determines if its farther away than the stop_distance
    returns True or False if the car should accelerate or not respectively

    :param   car:
    :return bool:
    """
    if not car['front-view']['distance-to-car']:
        return True
    else:
        if car['front-view']['distance-to-car'] > stop_distance:
            return True
        else:
            return False


def update_speed_factor(car):
    """
    handles logic for updating speed according to road curvature and car obstacles

    :param            car: dict
    :return: final_factor: double
    """
    obstacles = nav.FrontView(car)
    angles = obstacles.angles
    distance_to_node = car['front-view']['distance-to-node']
    distance_to_car = car['front-view']['distance-to-car']
    curvature_factor = road_curvature_factor(angles, distance_to_node)
    if distance_to_car:
        car_factor = car_obstacle_factor(distance_to_car)
        if distance_to_car > distance_to_node:
            final_factor = models.weigh_factors(
                car_factor, curvature_factor, distance_to_car, distance_to_node, free_distance
            )
        else:
            final_factor = car_factor
    else:
        final_factor = curvature_factor

    return abs(final_factor)


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
        if (stop_distance <= d) and (d <= free_distance):
            curvature_factor = math.log(d / (stop_distance * 2 * theta / math.pi)) / \
                               math.log(free_distance / (stop_distance * 2 * theta / math.pi))
        else:
            curvature_factor = 1

    return curvature_factor


def car_obstacle_factor(d):
    """
    calculates the speed factor (between 0 and 1) for road curvature

    Parameters
    _________
    :param                d: double:   distance to car in front_view

    Returns
    _______
    :return obstacle_factor: double:  factor by which to diminish speed
    """

    if (stop_distance <= d) and (d <= free_distance):
        obstacle_factor = math.log(d / stop_distance) / math.log(free_distance / stop_distance)
    else:
        if d < stop_distance:
            obstacle_factor = 0
        else:
            obstacle_factor = 1
    return obstacle_factor


def car_timer(car, dt):
    """
    This function increments a car's clock in all cases except when it is at its destination

    :param car:   dict
    :param  dt: double
    :return dt or 0: double: 0 only if car is at destination
    """
    if not np.isclose(car['position'], nav.get_position_of_node(car['destination']), atol=1).all():
        return dt
    else:
        return 0


def new_light_instructions(light, time_elapsed):
    """
    determines if it's time for a light to switch colors, then returns the new colors

    Parameters
    __________
    :param        light:   dict
    :param time_elapsed: double

    Returns
    _______
    :return new_instructions or None: list or None: list if time to switch, None if not
    """
    half_switch_time = light['switch-time']
    instructions = light['go']
    if np.isclose(time_elapsed, half_switch_time, rtol=1.0e-4):
        light['switch-counter'] += 1
        if light['switch-counter'] % 2:
            new_instructions = []
            for face in instructions:
                if face:
                    new_instructions.append(False)
                else:
                    new_instructions.append(True)
            return new_instructions
        else:
            return instructions
    else:
        return instructions


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
             'route-time': 0,
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
             'route-time': 0,
             'front-view': {'distance-to-car': 0, 'distance-to-node': 0, 'distance-to-red-light': 0},
             'origin': start_node,
             'destination': TEMP_dest_node
             }
        )
        cars[i]['path'] = np.array(nav.get_init_path(cars[i]))
        cars[i]['front-view']['distance-to-node'] = nav.FrontView(cars[i]).distances[0]

    return cars


def init_traffic_lights():
    """

    :return lights: list
    """
    light_nodes = nav.find_traffic_lights()

    lights = []

    for i, light in enumerate(light_nodes):
        node_id = light[0]
        degree = light[1]
        position = nav.get_position_of_node(node_id)
        go = [False, True] * degree * 2
        go = go[:degree]
        out_vectors = nav.determine_pedigree(node_id)
        pedigree = {
            ''
        }
        lights.append(
            {'position': position,
             'degree': degree,
             'go': go,
             'switch-counter': 0,
             'pedigree':
             }
        )
        lights[i]['switch-time'] = models.determine_traffic_light_timer()

    return lights

