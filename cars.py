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
        :param position:        array, double:              2D coordinates
        :param velocity:        array, double:              2D components
        :param acceleration:    array, double:              2D components
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

    def get_path(self):
        # send your location to the nav and return a path to your destination
        lines = nav.shortest_path_nx(self.position, self.destination)
        path = models.path_decompiler(lines)
        return path

    def front_view(self):
        


    def new_velocity(self, dt, G):





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








