"""
Description of module..

Traffic lights when color=red are obstacles just like cars
Cars slow down logarithmically as radius of road curvature gets smaller
Cars slow down for obstacles logarithmically as obstacles get closer, and stop at stop_distance
"""
import simulation as sim
import navigation as nav


class Cars:
    def __init__(self, init_state):
        """
        car objects are used for accessing and updating each car's parameters

        Parameters
        __________
        :param init_state: list:    each entry in the list is a car dict
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
        print('car time: {}'.format(self.time_elapsed))

        for i, car in enumerate(self.state):
            car['front-view']['distance-to-car'] = self.find_car_obstacles(car, i)
            car['front-view']['distance-to-node'] = nav.FrontView(car).distance_to_node()
            car['path'] = sim.update_path(car)
            car['velocity'] = sim.update_velocity(car)
            position = car['position']
            car['position'] = position + car['velocity'] * dt
            car['route-time'] += sim.car_timer(car, dt)

        return self.state

    def find_car_obstacles(self, car, i):
        """
        finds the distance to cars in the view for a specific car in the state

        :param       car:           dict: specific car of interest
        :param         i:            int: ID of the car in the state list
        :return distance: double or bool: returns False if no car in view
        """
        state = self.state.copy()
        state.pop(i)
        return nav.car_obstacles(state, car)


class TrafficLights:
    def __init__(self, light_state):
        """
        traffic light objects are used for finding, updating, and timing traffic light nodes
        """
        self.init_state = light_state
        self.state = self.init_state.copy()
        self.time_elapsed = 0

    def update(self, dt):
        """
        update the state of the traffic light

        :param dt:
        :return:
        """
        self.time_elapsed += dt

        for light in self.state:
            new_instructions = sim.new_light_instructions(light, self.time_elapsed)
            if new_instructions:
                light['go'] = new_instructions
            else:
                continue

        return self.state
