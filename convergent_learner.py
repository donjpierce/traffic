# python=3.6 requires using Qt4Agg backend for animation saving
import matplotlib
# matplotlib.use('Qt4Agg')
import models
import navigation as nav
from networkx import NetworkXNoPath
import numpy as np
import osmnx as ox
import pandas as pd

"""Piedmont, California"""
G = ox.load_graphml('piedmont.graphml')
G = ox.project_graph(G)
fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5)

# grab the dimensions of the figure
axis = ax.axis()


def init_custom_agent(n=1, fig_axis=axis, car_id=None, alternate_route=None):
    """
    This function initializes a singular car with custom origin and destination

    :param               n:          int
    :param        fig_axis:         list
    :param          car_id:  None or int
    :param alternate_route: None or list
    :return     cars_frame:    DataFrame
    """

    origin = 53085387
    dest = 53082621

    path = nav.get_init_path(origin, dest)
    route = nav.get_route(origin, dest)

    x, y = nav.get_position_of_node(origin)

    car = {'object': 'car',
           'x': x,
           'y': y,
           'vx': 0,
           'vy': 0,
           'route-time': 0,
           'origin': origin,
           'destination': dest,
           'route': route,
           'xpath': np.array([path[i][0] for i in range(len(path))]),
           'ypath': np.array([path[i][1] for i in range(len(path))]),
           'distance-to-car': 0,
           'distance-to-node': 0,
           'distance-to-red-light': 0}

    cars_data = [car]

    if alternate_route:
        cars_data[car_id]['route'], cars_data[car_id]['xpath'], cars_data[car_id]['ypath'] = alternate_route

    cars_frame = pd.DataFrame(cars_data)

    # determine binning and assign bins to cars
    cars_frame['xbin'], cars_frame['ybin'] = models.determine_bins(fig_axis, cars_frame)

    return cars_frame


def init_custom_lights(fig_axis, prescale=None):
    """
    traffic lights are initialized here

    :param   fig_axis:  list
    :param   prescale:   int
    :return    lights:  list
    """
    epsilon = 0.1  # a factor which forces the positions of the light faces to be close to the intersection

    lights_data = []

    node_id = 53119168

    try:
        out_vectors = np.array(nav.determine_pedigree(node_id))
    except NetworkXNoPath or ValueError:
        raise('Could not determine pedigree for light at node {}'.format(node_id))

    degree = len(out_vectors)
    x, y = nav.get_position_of_node(node_id)
    go = [False, True] * degree * 2
    go = go[:degree]

    light = {'object': 'light',
             'node': node_id,
             'degree': degree,
             'x': x,
             'y': y,
             'switch-counter': 0,
             'switch-time': models.determine_traffic_light_timer()
             }

    light['out-xpositions'] = [x + epsilon * out_vectors[j][0] for j in range(light['degree'])]
    light['out-ypositions'] = [y + epsilon * out_vectors[j][1] for j in range(light['degree'])]
    light['out-xvectors'] = [out_vectors[j][0] for j in range(light['degree'])]
    light['out-yvectors'] = [out_vectors[j][1] for j in range(light['degree'])]
    light['go-values'] = np.array([go[j] for j in range(light['degree'])])

    lights_data.append(light)

    lights = pd.DataFrame(lights_data)

    # determine binning and assign bins to lights
    lights['xbin'], lights['ybin'] = models.determine_bins(fig_axis, lights)

    # print('Number of traffic lights: {}'.format(len(lights)))
    return lights
