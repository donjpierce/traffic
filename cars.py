"""
Description of module..

Traffic lights when color=red are obstacles just like cars
Cars slow down exponentially as radius of road curvature gets smaller
Cars slow down for obstacles exponentially as obstacles get closer, and stop at stop_distance
"""
import navigation as nav
import simulation as sim
import models


class Car:
    def __init__(self, position, velocity, acceleration, front_view, destination):
        """
        car objects are be used for accessing and updating each car's parameters

        Parameters
        __________
        :param position:        tuple, double:              2D coordinates
        :param velocity:        tuple, double:              2D components
        :param acceleration:    tuple, double:              2D components
        :param front_view:      double:                     distance to next obstacle
        :param destination:     graph node ID, integer:     node ID from graphml graph
        """
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.front_view = front_view
        self.destination = destination

    def update(self, dt):
        """
        update the position of the car by a dt time step

        Parameters
        __________
        :param self:    object
        :param dt:      double

        Returns
        _______
        :return:
        """

        return

    def front_view(self):
        path = nav.get_path(self)
        next_graph_node = path[1]
        # distance_to_next_node =

        return 0


class State:
    def __init__(self, state):
        """
        the state is the description of all cars and is updated each time step

        Parameters
        __________
        :param state: dataframe
        """
        self.state = state
        self.elapsed_time = 0

    def update(self, dt):
        """
        update state by dt time step

        Parameters
        __________
        :param dt: float

        Return
        :return:
        """
        self.elapsed_time += dt








