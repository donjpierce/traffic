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
        :param init_state: list:    each entry in the list is a car dict
        """
        self.init_state = init_state
        self.state = self.init_state.copy()
        self.time_elapsed = 0

    def update(self, dt, light_conditions):
        """
        update the position of the car by a dt time step

        Parameters
        __________
        :param               dt:  double
        :param light_conditions:    list

        Returns
        _______
        :return:
        """
        self.time_elapsed += dt

        for i, car in enumerate(self.state):
            car['front-view']['distance-to-red-light'] = self.find_light_obstacles(car, light_conditions)
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
        :return distance: double or bool: returns None if no car in view
        """
        state = self.state.copy()
        state.pop(i)
        return nav.car_obstacles(state, car)

    def find_light_obstacles(self, car, light_conditions):
        """
        finds the distance to red lights in the view for a specific car in the state

        :param                     car:           dict: specific car of interest
        :param        light_conditions:           list:
        :return: distance_to_red_light: double or bool: returns None if no car in view
        """
        return nav.light_obstacles(car, light_conditions)


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
        if not self.time_elapsed % 0.5:
            print(self.time_elapsed)

        for light in self.state:
            new_instructions = sim.new_light_instructions(light, self.time_elapsed)
            if new_instructions:
                for i, instruction in enumerate(new_instructions):
                    light['pedigree'][i]['go'] = instruction
            else:
                continue

        return self.state
