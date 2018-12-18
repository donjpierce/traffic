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
# G = ox.load_graphml('sanfrancisco.graphml')
G = ox.project_graph(G)


class FrontView:
    def __init__(self, car, look_ahead_nodes=3):
        """
        take a car object and determines the obstacles it faces in its front_view

        :param car: dict
        :param look_ahead_nodes:
        """
        self.look_ahead_nodes = look_ahead_nodes
        self.car = car
        self.view = self.determine_view()
        self.angles = models.get_angles(self.view)
        self.obstacle_cars = 0
        self.distances = models.get_distances(self.view)

    def determine_view(self):
        """
        this method handles the exception where the path is shorter than look_ahead_nodes

        :return view: list of nodes immediately ahead of the car
        """
        if len(self.car['path']) > self.look_ahead_nodes:
            return [self.car['path'][i] for i in range(self.look_ahead_nodes)]
        else:
            return self.car['path']

    def distance_to_node(self):
        """
        Determines the distance to the most immediate node

        :return distance: double
        """
        position = self.car['position']
        next_node = np.array(self.upcoming_node_position())
        distance_vector = next_node - position
        distance = models.magnitude(distance_vector)
        return distance

    def upcoming_node_position(self):
        """
        Determines the coordinates of the next node in view

        :return view: tuple: returns upcoming node coords in the path
        """
        if len(self.view) <= 1:
            # if it's the end of the route, then the upcoming_node is simply the only node in view
            return self.view[0]

        space = models.upcoming_linspace(self.view)
        x_space = space[0]
        y_space = space[1]

        car_within_xlinspace = np.isclose(x_space, self.car['position'][0], rtol=1.0e-6).any()
        car_within_ylinspace = np.isclose(y_space, self.car['position'][1], rtol=1.0e-6).any()

        if car_within_xlinspace and car_within_ylinspace:
            return self.view[1]
        else:
            return self.view[0]

    def crossed_node_event(self):
        """
        Determines if the car has crossed a node, and advises simulation to change
        its velocity vector accordingly

        :return bool: True if the car is passing a node, False otherwise
        """
        car_near_xnode = np.isclose(self.view[0][0], self.car['position'][0], rtol=1.0e-6)
        car_near_ynode = np.isclose(self.view[0][1], self.car['position'][1], rtol=1.0e-6)

        if car_near_xnode and car_near_ynode:
            return True
        else:
            return False


def car_obstacles(state, car):
    """

    Parameters
    __________
    :param state:  list:  the entire car state with the car of interest removed
    :param   car:  dict:  the car of interest

    Returns
    _______
    :return distance: double or None (returns None if no car obstacle found)
    """
    obstacles = FrontView(car)
    space = models.upcoming_linspace(obstacles.view)
    x_space = space[0]
    y_space = space[1]

    obstacle_position = []
    for potential_obstacle in state:
        car_within_xlinspace = np.isclose(x_space, potential_obstacle['position'][0], rtol=1.0e-6).any()
        car_within_ylinspace = np.isclose(y_space, potential_obstacle['position'][1], rtol=1.0e-6).any()

        if car_within_xlinspace and car_within_ylinspace:
            obstacle_position.append(potential_obstacle['position'])

    if obstacle_position:
        first_obstacle = obstacle_position[0]
        x, y = first_obstacle[0], first_obstacle[1]
        vector = (x - car['position'][0], y - car['position'][1])
        distance = models.magnitude(vector)
        return distance
    else:
        return None


def find_culdesacs():
    """
    culdesacs are nodes with only one edge connection and which are not on the boundary of the OpenStreetMap

    :return culdesacs: list of node IDs
    """
    culdesacs = [key for key, value in G.graph['streets_per_node'].items() if value == 1]
    return culdesacs


def find_nodes(n):
    """
    returns n node IDs from the networkx graph

    :param      n: int
    :return nodes: list
    """
    nodes = []
    for node in G.nodes():
        nodes.append(node)
    return nodes[:n]


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


def get_init_path(car):
    """
    compiles a list of tuples which represents a route

    :param car: dict
    :return path: list where each entry is a tuple of tuples
    """
    lines = shortest_path_lines_nx(car)
    path = models.path_decompiler(lines)
    return path


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

    # yx_car_position = (car['position'][1], car['position'][0])
    # origin = ox.utils.get_nearest_node(G, yx_car_position)
    route = nx.shortest_path(G, car['origin'], car['destination'], weight='length')

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
