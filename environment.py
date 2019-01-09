from cars import Cars, TrafficLights
import navigation as nav
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
        self.route_time = 0
        self.cars_object = None
        self.lights_object = None
        self.dt = 1 / 1000
        self.stateview = nav.StateView(axis=self.axis, car_index=self.agent,
                                       cars=self.cars_object.state, lights=self.lights_object.state)

    def reset(self):
        """
        resets the environment
        :return s: state
        """
        # initialize the car and light state objects
        self.cars_object = Cars(sim.init_culdesac_start_location(self.N, self.axis), self.axis)
        self.lights_object = TrafficLights(sim.init_traffic_lights(self.axis, prescale=40), self.axis)
        state = self.stateview.determine_state()[0]
        state = state.index(True)
        return state

    def step(self, action):
        """
        This function runs a full simulation of a car from origin to destination
        (if action, then use the alternate route)

        :param action: int: 0 or 1
        :return:
        """

        if action:
            state, new_route, new_xpath, new_ypath = self.stateview.determine_state()
            self.cars_object.state.loc[self.agent]['route'] = new_route
            self.cars_object.state.loc[self.agent]['xpath'] = new_xpath
            self.cars_object.state.loc[self.agent]['ypath'] = new_ypath

        arrived = False
        while not arrived:
            remaining_nodes_in_route = self.cars_object.state.loc[self.agent]['route']
            if len(remaining_nodes_in_route) == 0:
                arrived = True
            self.lights_object.update(self.dt)
            self.cars_object.update(self.dt, self.lights_object.state)

        route_time = self.cars_object.state.loc[self.agent]['route-time']

        return
