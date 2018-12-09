"""
This module is the only one capable of referencing the map G
and thus contains methods for updating car position and finding path to car destination;
also contains methods for locating cars and intersections in the front_view
and calculating the curvature of the bend in the road for speed adjustments
"""
import numpy as np
import osmnx as ox
import networkx as nx

import models


G = ox.load_graphml('piedmont.graphml')


def get_path(car):
    """
    compiles a list of tuples which represents a route
    :param car: object
    :return path: list where each entry is a tuple of tuples
    """
    lines = shortest_path_lines_nx(car)
    path = models.path_decompiler(lines)
    return path


class Obstacles:
    def __init__(self, angles, distances, car, obstacle_cars, look_ahead_nodes=5):
        self.angles = angles
        self.distances = distances
        self.car = car
        self.obstacle_cars = obstacle_cars
        self.look_ahead_nodes = look_ahead_nodes

    def upcoming_angles(self):
        """
        Determines the road curvature immediately 5 nodes ahead ahead
        :param              self: object:
        :return: angles_in_view:   list: list of 3 angles corresponding to the 5-node curve
        """
        path = get_path(self.car)
        view = [path[i] for i in range(self.look_ahead_nodes)]
        angles_in_view = models.get_angles(view)
        return angles_in_view

    def upcoming_distances(self):
        """
        Determines the distances to the next 5 nodes
        :param                  self:  object:
        :return: distances_to_nodes:    list:   list of 4 distances corresponding to the 5-nodes ahead
        """
        path = get_path(self.car)
        view = [path[i] for i in range(self.look_ahead_nodes)]
        distances_in_view = models.get_distances(view)
        return distances_in_view


def find_culdesacs():
    """
    culdesacs are nodes with only one edge connection and which are not on the boundary of the OpenStreetMap

    :return culdesacs: list of node IDs
    """
    culdesacs = [key for key, value in G.graph['streets_per_node'].items() if value == 1]
    return culdesacs


def get_position_of_node(node):
    """
    Get latitude and longitude given node ID
    :param node:      graphml node ID
    :return position: array:    [latitude, longitude]
    """
    position = (G.nodes[node]['x'], G.nodes[node]['y'])
    return position


def shortest_path_lines_nx(Car):
    """
    uses the default shortest path algorithm available through networkx

    Parameters
    __________
    :param Car: object

    Returns
    _______
    :return lines:      list:   [(double, double), ...]:   each tuple represents the bend-point in a straight road
    """

    origin = ox.utils.get_nearest_node(G, Car.position)

    route = nx.shortest_path(G, origin, Car.destination, weight='length')

    # find the route lines
    edge_nodes = list(zip(route[:-1], route[1:]))
    lines = []
    for u, v in edge_nodes:
        # if there are parallel edges, select the shortest in length
        data = min(G.get_edge_data(u, v).values(), key=lambda x: x['length'])

        # if it has a geometry attribute (ie, a list of line segments)
        if 'geometry' in data:
            # add them to the list of lines to plot
            xs, ys = data['geometry'].xy
            lines.append(list(zip(xs, ys)))
        else:
            # if it doesn't have a geometry attribute, the edge is a straight
            # line from node to node
            x1 = G.nodes[u]['x']
            y1 = G.nodes[u]['y']
            x2 = G.nodes[v]['x']
            y2 = G.nodes[v]['y']
            line = ((x1, y1), (x2, y2))
            lines.append(line)

    return lines
