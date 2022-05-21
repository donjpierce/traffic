"""
Description of module...
"""
import math
import models
import navigation as nav
from networkx.exception import NetworkXNoPath
import numpy as np
import pandas as pd
import random

# fill the initial state with N cars
speed_limit = 1500
stop_distance = 20
free_distance = 60
default_acceleration = 4


def update_cars(cars, graph, dt):
    """
    This function shortens the stored path of a car after determining if the car crossed the next node in the path
    Then calculates the direction and magnitude of the velocity

    :param:       cars: dataframe
    :param:      graph: OGraph object from osm_request
    :param:         dt: double
    :return:     list: four Series's suitable for the main dataframe
    """
    new_route = []
    new_xpaths = []
    new_ypaths = []
    new_vx = []
    new_vy = []
    new_times = []

    for i, car in enumerate(cars.iterrows()):
        xpath, ypath = np.array(car[1]['xpath']), np.array(car[1]['ypath'])
        if xpath.any() and ypath.any():
            # add to route timer
            new_times.append(car[1]['route-time'] + dt)

            # initialize an obstacle scan of the frontal view
            frontview = nav.FrontView(car[1], graph, stop_distance=stop_distance)

            # determine if the car has just crossed a node
            if frontview.crossed_node_event():
                new_xpaths.append(car[1]['xpath'][1:])
                new_ypaths.append(car[1]['ypath'][1:])
            else:
                new_xpaths.append(car[1]['xpath'])
                new_ypaths.append(car[1]['ypath'])

            next_node = np.array(frontview.upcoming_node_position())
            position = np.array(frontview.position)

            velocity_direction = models.unit_vector(next_node - position)
            velocity = velocity_direction * speed_limit * update_speed_factor(car[1])

            # if the car has stalled and accelerate() returns True, then give it a push
            if np.isclose(0, velocity, atol=0.1).all() and accelerate(car[1]):
                velocity += default_acceleration

            new_vx.append(velocity[0])
            new_vy.append(velocity[1])
        else:
            # set empty paths
            new_xpaths.append([])
            new_ypaths.append([])
            # set null velocity
            new_vx.append(0)
            new_vy.append(0)
            # save final route time
            new_times.append(car[1]['route-time'])

    package = [pd.Series(new_route, dtype='float'), pd.Series(new_xpaths, dtype='object'),
               pd.Series(new_ypaths, dtype='object'), pd.Series(new_vx, dtype='float'),
               pd.Series(new_vy, dtype='float'), pd.Series(new_times, dtype='float')]

    return package


def accelerate(car):
    """
    determines if there is a car ahead or a red light. Returns True if the car should accelerate, False if not.

    :param   car: Series
    :return bool:
    """
    if not car['distance-to-red-light']:
        if not car['distance-to-car'] or car['distance-to-car'] > stop_distance:
            return True
        else:
            return False
    else:
        return False


def update_speed_factor(car):
    """
    handles logic for updating speed according to road curvature and car obstacles

    :param            car: Series
    :return: final_factor: double
    """
    frontview = nav.FrontView(car, stop_distance)
    angles = frontview.angles
    distance_to_node = car['distance-to-node']
    distance_to_car = car['distance-to-car']
    distance_to_red_light = car['distance-to-red-light']
    curvature_factor = road_curvature_factor(car, angles, distance_to_node)

    if distance_to_car and distance_to_red_light:
        if distance_to_car <= distance_to_red_light:
            final_factor = obstacle_factor(distance_to_car)
        else:
            final_factor = obstacle_factor(distance_to_red_light)
    else:
        if distance_to_car and not distance_to_red_light:
            car_factor = obstacle_factor(distance_to_car)
            if distance_to_car > distance_to_node:
                final_factor = models.weigh_factors(
                    car_factor, curvature_factor, distance_to_car, distance_to_node, free_distance
                )
            else:
                final_factor = car_factor
        else:
            if distance_to_red_light:
                final_factor = obstacle_factor(distance_to_red_light)
            else:
                final_factor = curvature_factor
    return abs(final_factor)


def road_curvature_factor(car, angle, d):
    """
    calculates the speed factor (between 0 and 1) for road curvature

    Parameters
    __________
    :param           car: Series
    :param         angle: double:  angles of road curvature ahead
    :param             d: double:  distance from car to next node

    Returns
    _______
    :return speed_factor: double:  factor by which to diminish speed
    """
    xpath = np.array(car['xpath'])
    if xpath.size == 1:
        # if it's the end of the path, treat the last node like a hard-stop intersection
        theta = math.pi / 2
    else:
        theta = angle

    if np.isclose(theta, 0, rtol=1.0e-1):
        curvature_factor = 1
    else:
        if (stop_distance < d) and (d <= free_distance):
            curvature_factor = math.log(d / (stop_distance * 2 * theta / math.pi)) / \
                               math.log(free_distance / (stop_distance * 2 * theta / math.pi))
        else:
            curvature_factor = 1
    return curvature_factor


def obstacle_factor(d):
    """
    calculates the speed factor (between 0 and 1) for road curvature

    Parameters
    _________
    :param                d: double:   distance to car in front_view

    Returns
    _______
    :return obstacle_factor: double:  factor by which to diminish speed
    """
    if (stop_distance < d) and (d <= free_distance):
        factor = math.log(d / stop_distance) / math.log(free_distance / stop_distance)
    else:
        if d <= stop_distance:
            factor = 0
        else:
            factor = 1
    return factor


def init_random_node_start_location(n, graph):
    """
    initializes n cars at n random nodes and sets their destinations as a culdesac

    :param      n:  int
    :param graph: object: OGraph object from osm_request
    :return state: dict
    """
    # TODO: combine this function with other car initialization functions using flags

    nodes = nav.find_nodes(graph, n)

    cars_data = []
    for i in range(n):
        if i < n - 1:
            origin = nodes[i]

            # random routes end at culdesacs
            # culdesacs = nav.find_culdesacs()
            # destination = culdesacs[i % len(culdesacs)]

            # random routes end at random places too
            random_index = round(random.random() * n)
            # print('len nodes')
            # print(len(nodes)))
            # print('random index')
            # print(random_index)
            destination = nodes[random_index]

            try:
                path = nav.get_init_path(graph, origin, destination)
                route = nav.get_route(graph, origin, destination)
            except NetworkXNoPath:
                print('No path between {} and {}.'.format(origin, destination))
                continue

            x, y = nav.get_position_of_node(graph, origin)

            car = {'object': 'car',
                   'x': x,
                   'y': y,
                   'vx': 0,
                   'vy': 0,
                   'route-time': 0,
                   'origin': origin,
                   'destination': destination,
                   'route': route,
                   'xpath': [path[i][0] for i in range(len(path))],
                   'ypath': [path[i][1] for i in range(len(path))],
                   'distance-to-car': 0,
                   'distance-to-node': 0,
                   'distance-to-red-light': 0}

            cars_data.append(car)

    cars = pd.DataFrame(cars_data)

    # determine binning and assign bins to cars
    axis = graph.axis
    xbins, ybins = np.arange(axis[0], axis[1], 200), np.arange(axis[2], axis[3], 200)
    x_indices, y_indices = np.digitize(cars['x'], xbins), np.digitize(cars['y'], ybins)
    cars['xbin'], cars['ybin'] = pd.Series(x_indices), pd.Series(y_indices)

    print('Number of cars: {}'.format(len(cars)))

    return cars


def init_culdesac_start_location(n, graph, car_id=None, alternate_route=None):
    """
    initializes N cars into N culdesacs

    Parameters
    __________
    :param               n:                    int
    :param graph: object: OGraph object from osm_request
    :param          car_id:            None or int: optional, int if you wish to prescribe an alternate route for car
    :param alternate_route:                   list: optional, list of alternate route nodes for provided car

    Returns
    _______
    :return cars:   dataframe
    """
    # TODO: combine this function with other car initialization functions using flags

    culdesacs = nav.find_culdesacs(graph)

    if n > len(culdesacs):
        raise ValueError('Number of cars greater than culdesacs to place them. '
                         'Choose a number less than {}'.format(len(culdesacs)))

    cars_data = []

    for i in range(n):
        # i = 17  # TEMP SETTING
        origin = culdesacs[i]
        destination = culdesacs[i + 1]
        """ START TEMP SETTINGS FOR ONE-CAR-ONE-ROUTE STUDY """
        # destination = 53028190
        """ END TEMP SETTINGS FOR ONE-CAR-ONE-ROUTE STUDY """
        try:
            path = nav.get_init_path(graph, origin, destination)
            route = nav.get_route(graph, origin, destination)
        except NetworkXNoPath:
            print('No path between {} and {}.'.format(origin, destination))
            continue

        position = nav.get_position_of_node(graph, origin)

        car = {'object': 'car',
               'x': position[0],
               'y': position[1],
               'vx': 0,
               'vy': 0,
               'route-time': 0,
               'origin': origin,
               'destination': destination,
               'route': route,
               'xpath': [path[i][0] for i in range(len(path))],
               'ypath': [path[i][1] for i in range(len(path))],
               'distance-to-car': 0,
               'distance-to-node': 0,
               'distance-to-red-light': 0}

        cars_data.append(car)

    if alternate_route:
        cars_data[car_id]['route'], cars_data[car_id]['xpath'], cars_data[car_id]['ypath'] = alternate_route

    cars = pd.DataFrame(cars_data)

    # determine binning and assign bins to cars
    cars['xbin'], cars['ybin'] = models.determine_bins(graph.axis, cars)

    # print('Number of cars: {}'.format(len(cars)))
    return cars


def init_traffic_lights(graph, prescale=10):
    """
    traffic lights are initialized here

    :param graph: object: OGraph object from osm_request
    :param prescale: int: percentage of intersections in graph to skip over and not create a light
    :return lights: list
    """
    epsilon = 0.3  # a factor which forces the positions of the light faces to be close to the intersection

    light_nodes = nav.find_traffic_lights(graph, prescale)

    lights_data = []

    for i, light in enumerate(light_nodes):

        node_id = light[0]

        try:
            out_vectors = np.array(nav.determine_pedigree(graph, node_id))
        except NetworkXNoPath or ValueError:
            print('Could not determine pedigree for light at node {}'.format(node_id))
            continue

        degree = len(out_vectors)
        position = nav.get_position_of_node(graph, node_id)
        go = [False, True] * degree * 2
        go = go[:degree]

        light = {'object': 'light',
                 'node': node_id,
                 'degree': degree,
                 'x': position[0],
                 'y': position[1],
                 'switch-counter': 0,
                 'switch-time': models.determine_traffic_light_timer(degree)
                 }

        light['out-xpositions'] = [position[0] + epsilon * out_vectors[j][0] for j in range(light['degree'])]
        light['out-ypositions'] = [position[1] + epsilon * out_vectors[j][1] for j in range(light['degree'])]
        light['out-xvectors'] = [out_vectors[j][0] for j in range(light['degree'])]
        light['out-yvectors'] = [out_vectors[j][1] for j in range(light['degree'])]
        light['go-values'] = np.array([go[j] for j in range(light['degree'])])

        lights_data.append(light)

    lights = pd.DataFrame(lights_data)

    # determine binning and assign bins to lights
    lights['xbin'], lights['ybin'] = models.determine_bins(graph.axis, lights)

    # print('Number of traffic lights: {}'.format(len(lights)))
    return lights
