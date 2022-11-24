"""
Example usage:

lights_object = TrafficLights(...)
cars = Cars(...)

dt, car_states, light_states = [], [], 1 / 1000
for i in range(100):
    light_state = lights_object.update(dt)
    light_states.append(lights_state)

    state = cars.update(dt=dt, lights_object.state)
    car_states.append(state)

"""
import simulation as sim
import models
import navigation as nav
import numpy as np


class Cars:
    def __init__(self, init_state, graph, serialize=False):
        """
        car objects are used for accessing and updating each car's parameters

        Parameters
        __________
        :param: init_state: dataframe:    each Series row is a car
        :param:      graph: object: OGraph object from osm_request
        :param:  serialize:   bool: write the state DataFrame to a local data store each update (very slow when True)
        """
        self.init_state = init_state
        self.state = self.init_state.copy()
        self.time_elapsed = 0
        self.lights = 0
        self.graph = graph
        self.serialize = serialize
        self.serialize_path = 'data_store'
        self.axis = self.graph.axis
        self.stop_distance = 5

    def write_state(self):
        """

        :return:
        """
        # write to local data store
        write_state = self.state.copy()
        # pre-process
        time_elapsed = str(round(self.time_elapsed, 4)).replace(".", "").zfill(4)

        write_state.to_parquet(
            f'{self.serialize_path}/dt={time_elapsed}/cars_state_at_{time_elapsed}.parquet.gz',
            engine='fastparquet',
            overwrite=True
        )
        return

    # TODO: profile
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
        # determine binning and assign bins to cars
        # TODO: don't re-sort every time-step. Only place cars in a new bin if their bin is about to change
        self.state['xbin'], self.state['ybin'] = models.determine_bins(self.axis, self.state)

        node_distances, car_distances, light_distances = self.find_obstacles()

        self.state['distance-to-node'] = node_distances
        self.state['distance-to-car'] = car_distances
        self.state['distance-to-red-light'] = light_distances

        self.state['route'], self.state['xpath'], self.state['ypath'], self.state['vx'], \
            self.state['vy'], self.state['route-time'] = sim.update_cars(self.state, self.graph, dt)

        self.state['x'] = self.state['x'] + self.state['vx'] * dt
        self.state['y'] = self.state['y'] + self.state['vy'] * dt

        if self.serialize:
            self.write_state()

        return self.state

    # TODO: optimize this function
    def find_obstacles(self):
        node_distances, car_distances, light_distances = [], [], []
        for car in self.state.iterrows():
            frontview = nav.FrontView(car[1], self.graph, stop_distance=self.stop_distance)
            node_distances.append(frontview.distance_to_node())
            car_distances.append(frontview.distance_to_car(self.state))
            light_distances.append(frontview.distance_to_light(self.lights))

        return node_distances, car_distances, light_distances


class TrafficLights:
    def __init__(self, light_state, graph):
        """
        traffic light objects are used for finding, updating, and timing traffic light nodes

        :param: light_state: list: each entry in the list is a light dictionary
        :param: graph: objectL OGraph object from osm_request
        """
        self.init_state = light_state
        self.state = self.init_state.copy()
        self.time_elapsed = 0
        self.xbins = np.arange(graph.axis[0], graph.axis[1], 200)
        self.ybins = np.arange(graph.axis[2], graph.axis[3], 200)

    def update(self, dt):
        """
        update the state of the traffic lights

        :param: dt:
        :return:
        """
        self.time_elapsed += dt
        time_to_switch = np.isclose(0, self.time_elapsed % self.state['switch-time'], rtol=1.0e-4)
        self.state['go-values'] = ~self.state['go-values'] * time_to_switch + self.state['go-values'] * ~time_to_switch
        return self.state
