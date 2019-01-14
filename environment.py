from animate import Animator
from cars import Cars, TrafficLights
import navigation as nav
import numpy as np
import simulation as sim

# additional testing module
import convergent_learner


class Env:
    def __init__(self, n, fig, ax, agent, dt, animate=False):
        """
        initializes an environment for a car in the system

        :param         n:       int: number of cars to simulate
        :param       fig:    figure: from matplotlib
        :param        ax:      axis: from matplotlib
        :param     agent:       int: the ID of the car (agent)
        :param   animate:      bool: if the environment is to be animated while learning
        """
        self.N = n
        self.num = None
        self.fig = fig
        self.ax = ax
        self.agent = agent
        self.dt = dt
        self.animate = animate
        self.animator = None
        self.axis = self.ax.axis()
        self.route_times = []
        self.cars_object = None
        self.lights_object = None
        # self.car_init_method = sim.init_culdesac_start_location
        # self.light_init_method = sim.init_traffic_lights
        self.car_init_method = convergent_learner.init_custom_agent
        self.light_init_method = convergent_learner.init_custom_lights

    def reset(self, num):
        """
        resets the environment

        :param    num: tuple: int, int
        :return state:   int
        """
        # initialize the car and light state objects
        init_cars = self.car_init_method(self.N, self.axis)
        self.cars_object = Cars(init_state=init_cars, axis=self.axis)
        init_light_state = self.light_init_method
        self.lights_object = TrafficLights(init_light_state(self.axis, prescale=40), self.axis)
        stateview = self.refresh_stateview()
        state = stateview.determine_state()[0]
        state = state.index(True)
        self.num = num
        self.animator = self.animator = Animator(fig=self.fig, ax=self.ax, cars_object=self.cars_object,
                                                 lights_object=self.lights_object, num=self.num)
        return state

    def refresh_stateview(self):
        """
        this function prepares a fresh depiction of what state the car is in

        :return stateview: object
        """
        stateview = nav.StateView(axis=self.axis, car_index=self.agent,
                                  cars=self.cars_object.state, lights=self.lights_object.state)
        return stateview

    def initialize_custom_reset(self, alternate_route):
        """
        resets the environment with a custom route for the agent

        :param alternate_route:   list: list of alternate route nodes for car agent
        :return          state:   list: initial state of agent
        """
        # initialize the car and light state objects
        init_car_state = self.car_init_method(self.N, self.axis, car_id=self.agent, alternate_route=alternate_route)
        self.cars_object = Cars(init_state=init_car_state, axis=self.axis)
        init_light_state = self.light_init_method(fig_axis=self.axis)
        self.lights_object = TrafficLights(light_state=init_light_state, axis=self.axis)
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
        stateview = self.refresh_stateview()
        state, new_route, new_xpath, new_ypath = stateview.determine_state()

        if action:
            new_state = self.initialize_custom_reset(alternate_route=(new_route, new_xpath, new_ypath))
        else:
            new_state = state.index(True)

        if self.animate:
            self.animator.reset(num)

        arrived = False
        i = 0
        while not arrived:
            i += 1
            arrived = self.simulation_step(i, self.animator)

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

        if num[0] < 2:
            reward = 0
        else:
            reward = self.route_times[num[0] - 1] - self.route_times[num[0]] + shortest_route_found_reward

        debug_report = 'Debug info: none'

        return new_state, reward, done, debug_report

    def simulation_step(self, i, animator):
        """
        make one step in the simulation

        :param         i: simulation step
        :param  animator: None or Animator object
        :return  arrived: bool
        """
        frontview = nav.FrontView(self.cars_object.state.loc[self.agent])
        if not frontview.end_of_route():
            if self.animate:
                animator.animate(i)
            else:
                self.lights_object.update(self.dt)
                self.cars_object.update(self.dt, self.lights_object.state)
            arrived = False
        else:
            arrived = True

        return arrived


