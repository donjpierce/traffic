"""
Description of module..

Traffic lights when color=red are obstacles just like cars
Cars slow down exponentially as radius of road curvature gets smaller
Cars slow down for obstacles exponentially as obstacles get closer, and stop at stop_distance
"""
import simulation as sim
import navigation as nav


class Cars:
    def __init__(self, init_state):
        """
        car objects are used for accessing and updating each car's parameters

        Parameters
        __________
        :param init_state: dataframe:    each entry in the dataframe is a car
        """
        self.init_state = init_state
        self.state = self.init_state.copy()
        self.time_elapsed = 0

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
            car['distance-to-car'] = self.find_car_obstacles()
            car['path'] = sim.update_path(car)
            car['velocity'] = sim.update_velocity(car)
            car['position'] = car['position'] + car['velocity'] * dt

        return self.state

    def find_car_obstacles(self):
        """
        finds the distance to cars in the view for a specific car in the state

        :return distance: double or bool (returns False if no car in view)
        """
        return nav.car_obstacles(self.state)


    # def traffic_lights(self):
