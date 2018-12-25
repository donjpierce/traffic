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
        :param init_state: dataframe:    each Series row is a car
        """
        self.init_state = init_state
        self.state = self.init_state.copy()
        self.time_elapsed = 0
        self.lights = 0

    def update(self, dt, lights, xy_range):
        """
        update the position of the car by a dt time step

        Parameters
        __________
        :param       dt:  double
        :param   lights:  dataframe
        :param xy_range:  tuple:    the geographical dimensions of the figure, and thus also the graph

        Returns
        _______
        :return self.state: dataframe
        """
        self.lights = lights
        self.time_elapsed += dt
        print(self.time_elapsed)

        node_distances, car_distances, light_distances = self.find_obstacles()

        self.state['distance-to-node'] = node_distances
        self.state['distance-to-car'] = car_distances
        self.state['distance-to-red-light'] = light_distances
        self.state['xpath'], self.state['ypath'], self.state['vx'], self.state['vy'] = sim.update_paths(self.state)
        self.state['x'] = self.state['x'] + self.state['vx'] * dt
        self.state['y'] = self.state['y'] + self.state['vy'] * dt
        # TODO: sim.car_timer performs another loop over all cars. So this should be combined with vel and paths
        self.state['route-time'] += sim.car_timer(self.state, dt)

        return self.state

    def find_obstacles(self):
        node_distances, car_distances, light_distances = [], [], []
        for car in self.state.iterrows():
            view = nav.FrontView(car[1])
            node_distances.append(view.distance_to_node())
            car_distances.append(view.distance_to_car(self.state))
            light_distances.append(view.distance_to_light(self.lights))

        return node_distances, car_distances, light_distances


class TrafficLights:
    def __init__(self, light_state):
        """
        traffic light objects are used for finding, updating, and timing traffic light nodes

        :param light_state: list: each entry in the list is a light dictionary
        """
        self.init_state = light_state
        self.state = self.init_state.copy()
        self.time_elapsed = 0

    def update(self, dt):
        """
        update the state of the traffic lights

        :param dt:
        :return:
        """
        self.time_elapsed += dt

        for light in self.state.iterrows():
            new_instructions = sim.new_light_instructions(light[1], self.time_elapsed)
            if new_instructions:
                for i, instruction in enumerate(new_instructions):
                    light[1]['pedigree'][i]['go'] = instruction
            else:
                continue

        return self.state
