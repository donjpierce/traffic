"""
This module references the map G and contains methods for updating car position and finding path to car destination
"""
import numpy as np
import osmnx as ox
import networkx as nx


G = ox.load_graphml('piedmont.graphml')


def get_position(node):
    """
    Get latitude and longitude given node ID
    :param node:      graphml node ID
    :return position: array:    [latitude, longitude]
    """
    position = np.array([G.nodes[node]['x'], G.nodes[node]['y']])
    return position


def shortest_path_nx(origin, destination):
    """
    uses the default shortest path algorithm available through networkx

    Parameters
    __________
    :param origin:      integer:    node ID
    :param destination: integer:    node ID

    Returns
    _______
    :return lines:      list:   [(double, double), ...]:   each tuple represents the bend-point in a straight road
    """
    route = nx.shortest_path(G, origin, destination, weight='length')

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
            line = [(x1, y1), (x2, y2)]
            lines.append(line)

    return lines
