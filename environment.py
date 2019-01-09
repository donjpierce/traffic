from cars import Cars, TrafficLights
from matplotlib import animation
import osmnx as ox
import simulation as sim

dt = 1 / 1000
N = 33

"""Lower Manhattan"""
# G = ox.load_graphml('lowermanhattan.graphml')
# G = ox.project_graph(G)
# fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5)

"""San Francisco"""
# G = ox.load_graphml('sanfrancisco.graphml')
# G = ox.project_graph(G)
# fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5)

"""Piedmont, California"""
G = ox.load_graphml('piedmont.graphml')
G = ox.project_graph(G)
fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5)

# grab the dimensions of the figure
axis = ax.axis()


class env:
    def __init__(self, n, axis):
        """

        :param    n: number of cars to simulate
        :param axis: dimensions of the figure
        """
        self.N = n
        self.axis = axis

    def reset(self):
        """
        resets the environment
        :return s: state
        """
        # initialize the car and light state objects
        cars_object = Cars(sim.init_culdesac_start_location(N, axis), axis)
        lights_object = TrafficLights(sim.init_traffic_lights(axis, prescale=40), axis)
