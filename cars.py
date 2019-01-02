"""
Description of module..

Traffic lights when color=red are obstacles just like cars
Cars slow down exponentially as radius of road curvature gets smaller
Cars slow down for obstacles exponentially as obstacles get closer, and stop at stop_distance
"""
import simulation as sim
import navigation as nav
import numpy as np
import pandas as pd

# TODO: using machine learning will require logging a bunch of data
# TODO: start thinking about what data is important to log for your training set


class Cars:
    def __init__(self, init_state, axis):
        """
        car objects are used for accessing and updating each car's parameters

        Parameters
        __________
        :param init_state: dataframe:    each Series row is a car
        """
        self.init_state = init_state
        self.state = self.init_state.copy()
        self.time_elapsed = 0
        self.lights = 0
        self.xbins = np.arange(axis[0], axis[1], 200)
        self.ybins = np.arange(axis[2], axis[3], 200)

    def update(self, dt, lights):
        """
        update the position of the car by a dt time step

        Parameters
        __________
        :param       dt:  double
        :param   lights:  dataframe

        Returns
        _______
        :return self.state: dataframe
        """
        self.lights = lights
        self.time_elapsed += dt
        print(self.time_elapsed)

        # determine binning and assign bins to cars
        # TODO: don't re-sort every time-step. Only place cars in a new bin if their bin is about to change
        x_indices, y_indices = np.digitize(self.state['x'], self.xbins), np.digitize(self.state['y'], self.ybins)
        self.state['xbin'], self.state['ybin'] = pd.Series(x_indices), pd.Series(y_indices)

        node_distances, car_distances, light_distances = self.find_obstacles()

        self.state['distance-to-node'] = node_distances
        self.state['distance-to-car'] = car_distances
        self.state['distance-to-red-light'] = light_distances

        self.state['route'], self.state['xpath'], self.state['ypath'], self.state['vx'], \
            self.state['vy'], self.state['route-time'] = sim.update_cars(self.state, dt)

        """
            Here is where we will grab the new route from nav.build_new_route() if a car (agent)
            decides to reroute
        """

        self.state['x'] = self.state['x'] + self.state['vx'] * dt
        self.state['y'] = self.state['y'] + self.state['vy'] * dt

        return self.state

    def find_obstacles(self):
        node_distances, car_distances, light_distances = [], [], []
        for car in self.state.iterrows():
            frontview = nav.FrontView(car[1])
            node_distances.append(frontview.distance_to_node())
            car_distances.append(frontview.distance_to_car(self.state))
            light_distances.append(frontview.distance_to_light(self.lights))

        return node_distances, car_distances, light_distances


class TrafficLights:
    def __init__(self, light_state, axis):
        """
        traffic light objects are used for finding, updating, and timing traffic light nodes

        :param light_state: list: each entry in the list is a light dictionary
        """
        self.init_state = light_state
        self.state = self.init_state.copy()
        self.time_elapsed = 0
        self.xbins = np.arange(axis[0], axis[1], 200)
        self.ybins = np.arange(axis[2], axis[3], 200)

    def update(self, dt):
        """
        update the state of the traffic lights

        :param dt:
        :return:
        """
        self.time_elapsed += dt
        time_to_switch = np.isclose(self.time_elapsed, self.state['switch-time'], rtol=1.0e-4)
        self.state['go-values'] = ~self.state['go-values'] * time_to_switch + self.state['go-values'] * ~time_to_switch

        return self.state
