import numpy as np

import navigation as nav
import simulation as sim
from animate import Animator
from cars import Cars, TrafficLights

# additional testing module
# import convergent_learner


class Env:
    def __init__(self, n, graph, agent, dt, animate=False):
        """
        initializes an environment for a car in the system

        :param         n:       int: number of cars to simulate
        :param     graph:  OGraph object from
        :param     agent:       int: the ID of the car (agent)
        :param   animate:      bool: if the environment is to be animated while learning
        """
        self.N = n
        self.num = None
        self.graph = graph
        self.fig, self.ax = self.graph.fig, self.graph.ax
        self.agent = agent
        self.dt = dt
        self.animate = animate
        self.animator = None
        self.axis = self.ax.axis()
        self.route_times = []
        self.car_init_method = sim.init_random_node_start_location
        # self.car_init_method = sim.init_culdesac_start_location
        self.light_init_method = sim.init_traffic_lights
        # self.car_init_method = convergent_learner.init_custom_agent
        # self.light_init_method = convergent_learner.init_custom_lights
        self.cars_object = Cars(self.car_init_method(self.N, self.graph), self.graph)
        self.lights_object = TrafficLights(
            self.light_init_method(self.graph, prescale=40), self.graph
        )
        self.high = 10
        self.low = 2
        self.shortest_route_thresh = 5

    def reset(self, num):
        """
        resets the environment

        :param    num: tuple: int, int
        :return state:   int
        """
        # initialize cars every reset
        init_cars = self.car_init_method(n=self.N, graph=self.graph)
        self.cars_object = Cars(init_state=init_cars, graph=self.graph)
        stateview = self.refresh_stateview()
        state = stateview.determine_state()[0]
        state = state.index(True)

        if self.animate:
            # init animator
            self.num = num
            self.animator = Animator(
                fig=self.fig,
                ax=self.ax,
                cars_object=self.cars_object,
                lights_object=self.lights_object,
                num=self.num,
                focus=self.agent,
            )

        return state

    def refresh_stateview(self):
        """
        this function prepares a fresh depiction of what state the car is in

        :return stateview: object
        """
        stateview = nav.StateView(
            graph=self.graph,
            car_index=self.agent,
            cars=self.cars_object.state,
            lights=self.lights_object.state,
        )
        return stateview

    def initialize_custom_reset(self, alternate_route):
        """
        resets the environment with a custom route for the agent

        :param alternate_route:   list: list of alternate route nodes for car agent
        :return          state:   list: initial state of agent
        """
        # initialize the car and light state objects
        init_car_state = self.car_init_method(
            graph=self.graph,
            n=self.N,
            car_id=self.agent,
            alternate_route=alternate_route,
        )
        self.cars_object = Cars(init_state=init_car_state, graph=self.graph)

        if self.animate:
            # init animator
            self.animator = Animator(
                fig=self.fig,
                ax=self.ax,
                cars_object=self.cars_object,
                lights_object=self.lights_object,
                num=self.num,
            )

        stateview = self.refresh_stateview()
        state = stateview.determine_state()[0]
        state = state.index(True)
        return state

    def step(self, action, num):
        """
        This function runs a full simulation of a car from origin to destination
        (if action, then use the alternate route)

        :param                      action:   int: 0 or 1
        :param                         num: tuple: the simulation number out of the total number of simulations
        :return new_state, reward, done, _:  list: the end of the return is free to contain debugging info
        """
        debug_report = []

        if self.animate:
            self.animator.reset(self.num)

        stateview = self.refresh_stateview()
        state, new_route, new_xpath, new_ypath = stateview.determine_state()

        if action:
            new_state = self.initialize_custom_reset(
                alternate_route=(new_route, new_xpath, new_ypath)
            )
        else:
            new_state = state.index(True)

        # Run the simulation until the car reaches the destination
        arrived = False
        i = 0
        while not arrived:
            arrived = self.simulation_step(i)
            i += 1

        route_time = self.cars_object.state.loc[self.agent]["route-time"]
        self.route_times.append(route_time)
        # TODO: need new way of identifying shortest route time.
        if len(self.route_times) < self.shortest_route_thresh:
            shortest_route_found_reward = 0
            done = False
        elif np.isclose(
            0, self.route_times[-1] - np.min(self.route_times), atol=5 * self.dt
        ).all():
            """
            If the route time achieved after the simulation is within 5 x dt second of the minimum time achieved.
            Define this environment condition as having found the shortest route (locally).
            """
            shortest_route_found_reward = self.high
            done = True
        else:
            shortest_route_found_reward = 0
            done = False

        if num[0] < 1:
            reward = 0
        else:
            time_delta = (
                self.route_times[num[0] - 1]
                - self.route_times[num[0]]
                + shortest_route_found_reward
            )
            if time_delta > 0:
                reward = time_delta
            else:
                reward = 0

        return new_state, reward, done, debug_report

    def simulation_step(self, i):
        """
        make one step in the simulation and determine if the car has arrived at the destination

        :param         i: simulation step
        :return  arrived: bool
        """
        frontview = nav.FrontView(self.cars_object.state.loc[self.agent], self.graph)
        end_of_route = frontview.end_of_route()
        if not end_of_route:
            if self.animate:
                self.animator.animate(i)
            else:
                self.lights_object.update(self.dt)
                self.cars_object.update(self.dt, self.lights_object.state)
            arrived = False
        else:
            arrived = True

        return arrived
