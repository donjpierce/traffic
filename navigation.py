"""
This module is the only one capable of referencing the map G
and thus contains methods for updating car position and finding path to car destination;
also contains methods for locating cars and intersections in the front_view
and calculating the curvature of the bend in the road for speed adjustments
"""
import models
import networkx as nx
import numpy as np
import osmnx as ox


G = ox.load_graphml('piedmont.graphml')
G = ox.project_graph(G)


def get_path(car):
    """
    compiles a list of tuples which represents a route

    :param car: dict
    :return path: list where each entry is a tuple of tuples
    """
    lines = shortest_path_lines_nx(car)
    path = models.path_decompiler(lines)
    return path


class FrontView:
    def __init__(self, car, look_ahead_nodes=3):
        """
        take a car object and determines the obstacles it faces in its front_view

        :param car: dict
        :param look_ahead_nodes:
        """
        self.car = car
        self.angles = 0
        self.distances = 0
        self.obstacle_cars = 0
        self.path = get_path(self.car)
        self.look_ahead_nodes = look_ahead_nodes
        self.view = [self.path[i] for i in range(self.look_ahead_nodes)]

    def upcoming_angles(self):
        """
        Determines the road curvature immediately 5 nodes ahead ahead

        :return: angles_in_view:   list: list of 3 angles corresponding to the 5-node curve
        """
        angles_in_view = models.get_angles(self.view)
        self.angles = angles_in_view
        return angles_in_view

    def upcoming_distances(self):
        """
        Determines the distances to the next 5 nodes

        :return: distances_to_nodes:    list:   list of 4 distances corresponding to the 5-nodes ahead
        """
        distances_in_view = models.get_distances(self.view)
        self.distances = distances_in_view
        return distances_in_view

    def upcoming_node_position(self):
        """
        Determines the coordinates of the next nodes in view

        :return view: list of node coordinates in view
        """
        return self.view[1]


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
    # note that the x and y coordinates of the G.nodes are flipped
    # this is possibly an issue with the omnx G.load_graphml method
    # a correction is to make the position tuple be (y, x) as below
    position = np.array([G.nodes[node]['x'], G.nodes[node]['y']])
    return position


def shortest_path_lines_nx(car):
    """
    uses the default shortest path algorithm available through networkx

    Parameters
    __________
    :param car: dict

    Returns
    _______
    :return lines: list:
        [(double, double), ...]:   each tuple represents the bend-point in a straight road
    """

    yx_car_position = (car['position'][1], car['position'][0])
    origin = ox.utils.get_nearest_node(G, yx_car_position)
    route = nx.shortest_path(G, origin, car['destination'], weight='length')

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
