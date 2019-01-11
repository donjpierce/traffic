# python=3.6 requires using Qt4Agg backend for animation saving
import matplotlib
matplotlib.use('Qt4Agg')
import models
import navigation as nav
import osmnx as ox
import pandas as pd

"""Piedmont, California"""
G = ox.load_graphml('piedmont.graphml')
G = ox.project_graph(G)
fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5)

# grab the dimensions of the figure
axis = ax.axis()


def init_custom_agent(fig_axis, car_id=None, alternate_route=None):
    """
    This function initializes a singular car with custom origin and destination

    :param        fig_axis:         list
    :param          car_id:  None or int
    :param alternate_route: None or list
    :return     cars_frame:    DataFrame
    """

    origin = 53073689
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
           'xpath': [path[i][0] for i in range(len(path))],
           'ypath': [path[i][1] for i in range(len(path))],
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

