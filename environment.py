from cars import Cars, TrafficLights
import navigation as nav
import numpy as np
import simulation as sim


class Env:
    def __init__(self, n, fig_axis, agent):
        """
        initializes an environment for a car in the system

        :param         n:       int: number of cars to simulate
        :param  fig_axis: quadruple: dimensions of the figure
        :param     agent:       int: the ID of the car (agent)
        """
        self.N = n
        self.axis = fig_axis
        self.agent = agent
        self.route_times = []
        self.cars_object = None
        self.lights_object = None
        self.dt = 1 / 1000

    def refresh_stateview(self):
        """
        this function prepares a fresh depiction of what state the car is in

        :return stateview: object
        """
        stateview = nav.StateView(axis=self.axis, car_index=self.agent,
                                  cars=self.cars_object.state, lights=self.lights_object.state)
        return stateview

    def reset(self):
        """
        resets the environment

        :return s: state
        """
        # initialize the car and light state objects
        self.cars_object = Cars(sim.init_culdesac_start_location(self.N, self.axis), self.axis)
        self.lights_object = TrafficLights(sim.init_traffic_lights(self.axis, prescale=40), self.axis)
        stateview = self.refresh_stateview()
        state = stateview.determine_state()[0]
        state = state.index(True)
        return state

    def step(self, action, num):
        """
        This function runs a full simulation of a car from origin to destination
        (if action, then use the alternate route)

        :param                      action:  int: 0 or 1
        :param                         num:  int: the iteration number
        :return new_state, reward, done, _: list: the end of the return is free to contain debugging info
        """

        if action:
            stateview = self.refresh_stateview()
            state, new_route, new_xpath, new_ypath = stateview.determine_state()
            self.cars_object.state.loc[self.agent]['route'] = new_route
            self.cars_object.state.loc[self.agent]['xpath'] = new_xpath
            self.cars_object.state.loc[self.agent]['ypath'] = new_ypath

        arrived = False
        while not arrived:
            time = self.cars_object.time_elapsed
            if time % 5.000 == 0:
                print('Running simulation. t = {}'.format(time))
            remaining_nodes_in_route = self.cars_object.state.loc[self.agent]['route']
            if len(remaining_nodes_in_route) == 0:
                arrived = True
            self.lights_object.update(self.dt)
            self.cars_object.update(self.dt, self.lights_object.state)

        stateview = self.refresh_stateview()
        new_state, route, xpath, ypath = stateview.determine_state()
        route_time = self.cars_object.state.loc[self.agent]['route-time']
        self.route_times.append(route_time)
        latest_two_times = [self.route_times[-i] for i in range(2)]
        if len(self.route_times) < 3:
            done = False
            shortest_route_found_reward = 0
        elif np.isclose(latest_two_times, np.min(self.route_times), atol=1).all():
            """
            The latest two route times are within 1 second of the minimum time achieved.
            Define this environment condition as having found the shortest route. 
            """
            shortest_route_found_reward = 100
            done = True
        else:
            done = False
            shortest_route_found_reward = 0

        if num < 2:
            reward = 0
        else:
            reward = self.route_times[num - 1] - self.route_times[num] + shortest_route_found_reward

        return
