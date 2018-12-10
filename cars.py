"""
Description of module..

Traffic lights when color=red are obstacles just like cars
Cars slow down exponentially as radius of road curvature gets smaller
Cars slow down for obstacles exponentially as obstacles get closer, and stop at stop_distance
"""
import navigation as nav
import simulation as sim
import models


class Cars:
    def __init__(self, init_state):
        """
        car objects are be used for accessing and updating each car's parameters

        Parameters
        __________
        :param init_state: dataframe:    each entry in the dataframe is a car
        """
        self.init_state = init_state
        self.state = self.init_state.copy()
        self.time_elapsed = 0
        self.positions = self.state['position']
        self.velocities = self.state['velocity']
        self.accelerations = self.state['acceleration']

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
        self.time_elapsed += dt

        for car in self.state:
            speed_factor = sim.update_speed_factor(car)
            velocity = car['velocity']
            position = car['position']
            acceleration = car['acceleration']



            car['acceleration'] =

        return 0








