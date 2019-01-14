"""
This module is the only one capable of referencing the map G
and thus contains methods for updating car position and finding path to car destination;
also contains methods for locating cars and intersections in the front_view
and calculating the curvature of the bend in the road for speed adjustments
"""
import models
import networkx as nx
import numpy as np
import osmnx as ox


G = ox.load_graphml('piedmont.graphml')
# G = ox.load_graphml('sanfrancisco.graphml')
# G = ox.load_graphml('lowermanhattan.graphml')
G = ox.project_graph(G)


class FrontView:
    def __init__(self, car, stop_distance=5, look_ahead_nodes=3):
        """
        take a car Series and determines the obstacles it faces in its frontal view

        :param              car: Series row of the main dataframe
        :param    stop_distance: int
        :param look_ahead_nodes: int
        """
        self.stop_distance = stop_distance
        self.look_ahead_nodes = look_ahead_nodes
        self.car = car
        self.position = car['x'], car['y']
        self.view = self.determine_view()
        self.angles = models.get_angles(self.view)

    def determine_view(self):
        """
        this method handles the exception where the path is shorter than look_ahead_nodes

        :return view: list or bool: list of nodes immediately ahead of the car or False if end of route
        """
        xpath, ypath = np.array(self.car['xpath']), np.array(self.car['ypath'])
        if xpath.any() and ypath.any():
            x, y = self.car['xpath'][:self.look_ahead_nodes], self.car['ypath'][:self.look_ahead_nodes]
            return [(x[i], y[i]) for i in range(len(x))]
        else:
            return False

    def distance_to_car(self, cars):
        """
        dispatches a car Series into another nav function and retrieves the distance to a car obstacle if there is one

        :param      cars: Dataframe of cars
        :return distance:
        """
        return car_obstacles(self, cars)

    def distance_to_light(self, lights):
        """
        dispatches a car Series into another nav function and retrieves the distance to a red light if there is one

        :param    lights: Dataframe of lights
        :return distance:
        """
        return light_obstacles(self, lights)

    def distance_to_node(self):
        """
        Determines the distance to the most immediate node

        :return distance: double
        """
        next_node = np.array(self.upcoming_node_position())
        distance_vector = next_node - self.position
        distance = models.magnitude(distance_vector)
        return distance

    def upcoming_node_position(self):
        """
        Determines the coordinates of the next node in view

        :return view: tuple: returns upcoming node coords in the path
        """
        if self.view:
            if self.crossed_node_event():
                if len(self.view) >= 2:
                    return self.view[1]
                else:
                    return get_position_of_node(self.car['destination'])
            else:
                return self.view[0]
        else:
            # end of route
            return get_position_of_node(self.car['destination'])

    def crossed_node_event(self):
        """
        Determines if the car has crossed a node, and advises simulation to change
        its velocity vector accordingly

        :return bool: True if the car is passing a node, False otherwise
        """
        car_near_xnode = np.isclose(self.view[0][0], self.car['x'], rtol=1.0e-6)
        car_near_ynode = np.isclose(self.view[0][1], self.car['y'], rtol=1.0e-6)

        if car_near_xnode and car_near_ynode:
            return True
        else:
            return False

    def end_of_route(self):
        """
        Determines if the car has reached the end of the route

        :return bool: False if not, True if car is at the end of its root
        """
        xdest, ydest = get_position_of_node(self.car['destination'])
        car_near_xdest = np.isclose(xdest, self.car['x'], atol=1)
        car_near_ydest = np.isclose(ydest, self.car['y'], atol=1)

        if car_near_xdest and car_near_ydest:
            return True
        else:
            return False


class StateView:
    def __init__(self, axis, car_index, cars, lights):
        """
        the reinforcement learning agent object

        :param      axis:
        :param car_index:
        :param      cars: DataFrame
        :param    lights: DataFrame
        """
        self.axis = axis
        self.cars = cars
        self.lights = lights
        self.index = car_index
        self.car = cars.loc[self.index]
        self.route = np.array(self.car['route'])
        self.eta = eta(self.car, self.lights)
        self.max_cars = 10  # the number of cars in a bin for the bin to be considered 'congested'
        self.speed_limit = 250

    def determine_state(self):
        """
        this method gathers information about the car's route, and determines which state the car is in

        :return state, new_route, new_xpath, new_ypath
        """
        if self.route.size > 0:
            # get light IDs in the route
            light_locs = self.get_lights_in_route()
            # get congested bins
            traffic_nodes = self.get_traffic_nodes()

            if light_locs or traffic_nodes:
                if light_locs and traffic_nodes:
                    """ If there are both types of obstacles, decide to reroute around the closest one """
                    long_light_ind = np.where(self.route == self.lights.loc[light_locs[-1]]['node'])[0][0]
                    first_traffic_node_ind = np.where(self.route == traffic_nodes[0])[0][0]
                    if long_light_ind <= first_traffic_node_ind:
                        # light comes first in route
                        return self.bulk(light_locs)
                    else:
                        # car comes first in route
                        return self.bulk(traffic_nodes)
                elif light_locs and not traffic_nodes:
                    # there are only lights are in route
                    return self.bulk(light_locs)
                elif traffic_nodes and not light_locs:
                    # there is only traffic in the route
                    return self.bulk(traffic_nodes)

            else:
                # there are no obstacles along the current route STATE 7      <---------
                state = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
                return state, self.route, self.car['xpath'], self.car['ypath']
        else:
            # the car has arrived at the destination STATE 10        <---------
            state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
            return state, self.route, self.car['xpath'], self.car['ypath']

    def bulk(self, light_locs=None, traffic_nodes=None):
        """
        this method determines whether the agent is in any one of states 1-6, 8, or 9

        :param    light_locs: None or list
        :param traffic_nodes: None or list
        :return state, new_route, new_xpath, new_ypath:
        """
        if light_locs:
            # re-route around light with longest switch-time (last light in array due to sorting)
            traffic, avoid_node = 0, self.lights.loc[light_locs[-1]]['node']
            new_route, new_xpath, new_ypath, detour = self.find_alternate_route(avoid_node, traffic)
        else:
            traffic, avoid_node = len(traffic_nodes), traffic_nodes[0]
            new_route, new_xpath, new_ypath, detour = self.find_alternate_route(avoid_node, traffic)

        """
        Calculate the length of the detour and the length of  
        the stretch of the original route which was avoided by the detour:
        """
        detour_length = sum([G.get_edge_data(detour[i], detour[i + 1])[0]['length']
                             for i in range(len(detour) - 1)])
        departure_ind = np.where(self.route == detour[0])[0][0]
        return_ind = np.where(self.route == detour[-1])[0][0]
        span = return_ind - departure_ind
        original_length = sum([G.get_edge_data(self.route[departure_ind + i],
                                               self.route[departure_ind + i + 1])[0]['length']
                               for i in range(span + 1)])
        if detour_length <= 2 * original_length:
            # detour is short
            obstacles_in_detour = np.array([self.get_lights_in_route(route=detour),
                                            self.get_traffic_nodes(route=detour)]).any()
            if not obstacles_in_detour:
                # there are no obstacles in the detour
                if light_locs:
                    # STATE 2       <---------
                    state = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
                else:
                    # STATE 1       <---------
                    state = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                return state, new_route, new_xpath, new_ypath
            else:
                # there are obstacles in the detour
                if light_locs:
                    # STATE 4       <---------
                    state = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
                else:
                    # STATE 3       <---------
                    state = [0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
                return state, new_route, new_xpath, new_ypath
        else:
            # detour is long
            obstacles_in_detour = np.array([self.get_lights_in_route(route=detour),
                                            self.get_traffic_nodes(route=detour)]).any()
            if not obstacles_in_detour:
                # there are no obstacles in the detour
                if light_locs:
                    # STATE 6       <---------
                    state = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0]
                else:
                    # STATE 5
                    state = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0]
                return state, new_route, new_xpath, new_ypath
            else:
                # there are obstacles in the detour
                if light_locs:
                    # STATE 9       <---------
                    state = [0, 0, 0, 0, 0, 0, 0, 0, 1, 0]
                else:
                    # STATE 8       <---------
                    state = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
                return state, new_route, new_xpath, new_ypath

    def find_alternate_route(self, avoid, traffic=0):
        """
        Uses build_new_route to find alternate routes

        :param   avoid: first node to avoid
        :param traffic: number of proceeding nodes to avoid (default 0 if avoid node is a traffic light)
        :return new_route, new_xpath, new_ypath:
        """
        new_route, new_xpath, new_ypath, detour = [], [], [], []
        found_route = False
        i = 0
        while not found_route:
            i += 1
            if i == 10:
                print('Could not find alternate route for car {}'.format(self.index))
                break

            reroute_node = self.route[np.where(self.route == avoid)[0][0] - i]

            # Determine in which direction to reroute
            dv_table = self.dv_table(reroute_node)
            direction = dv_table['potential-nodes'].loc[dv_table.index[dv_table['sum-distances'].idxmin()]]

            # get new route around obstacle
            data = build_new_route(self.route, reroute_node, direction, traffic, avoid)
            if data:
                new_route, new_xpath, new_ypath, detour = data
                found_route = True

        return new_route, new_xpath, new_ypath, detour

    def get_lights_in_route(self, route=None):
        """
        this method returns the IDs of the traffic lights anywhere along the route

        :param       route: optionally, provide a route other than the original route in which to check for lights
        :return light_locs: a list of light IDs
        """
        if not route:
            route = self.route
        light_locs = np.array([np.where(self.lights['node'] == node)[0][0] for node in route
                               if (node == self.lights['node']).any()])

        # sort lights by switch-time
        light_locs = [time for time in self.lights['switch-time'].argsort() if (time == light_locs).any()]

        if not light_locs:
            return None
        else:
            return light_locs

    def get_traffic_nodes(self, route=None):
        """
        this method returns the (xbin, ybin) pair of a bins which are considered to be congested with traffic

        :param          route: optionally, provide a route other than the original route in which to check for traffic
        :return traffic_nodes: list: list of nodes
        """
        traffic_nodes = []
        xbins, ybins = self.get_bins_in_route(route)
        xbin_points = np.arange(self.axis[0], self.axis[1], 200)
        ybin_points = np.arange(self.axis[2], self.axis[3], 200)
        for xbin, ybin in zip(xbins, ybins):
            for i, (cars_xbin, cars_ybin) in enumerate(zip(self.cars['xbin'], self.cars['ybin'])):
                if (xbin, ybin) == (cars_xbin, cars_ybin):
                    in_xy_bin = (np.digitize(self.car['xpath'], xbin_points) == xbin) & \
                                (np.digitize(self.car['ypath'], ybin_points) == ybin)

                    x_stretch = (self.car['xpath'] * in_xy_bin)[np.nonzero(self.car['xpath'] * in_xy_bin)]
                    y_stretch = (self.car['ypath'] * in_xy_bin)[np.nonzero(self.car['ypath'] * in_xy_bin)]

                    in_xpath = np.isclose(self.cars.loc[i]['x'], x_stretch, rtol=1e-6).any()
                    in_ypath = np.isclose(self.cars.loc[i]['y'], y_stretch, rtol=1e-6).any()
                    if in_xpath and in_ypath:
                        traffic_nodes.append(self.cars.loc[i]['route'][0])

        if len(traffic_nodes) > self.max_cars:
            traffic_nodes = models.clean_list(traffic_nodes)
            return traffic_nodes
        else:
            return None

    def get_bins_in_route(self, route=None):
        """
        this method parses the route and returns a list of xbins and ybins through which the route passes

        :param         route: list: optionally, provide a route other than the original
        :return xbins, ybins
        """
        if not route:
            route = self.route
        xbins, ybins = np.arange(self.axis[0], self.axis[1], 200), np.arange(self.axis[2], self.axis[3], 200)
        x_inds, y_inds = [], []
        for node in route:
            x, y = get_position_of_node(node)
            x_inds.append(np.digitize(x, xbins))
            y_inds.append(np.digitize(y, ybins))

        x_inds, y_inds = np.array(x_inds), np.array(y_inds)

        # remove double-counted bins from result
        xbins, ybins = [], []
        for i in range(len(x_inds)):
            if i < len(x_inds) - 1:
                if (x_inds[i] == x_inds[i + 1]) and (y_inds[i] == y_inds[i + 1]):
                    continue
                else:
                    xbins.append(x_inds[i])
                    ybins.append(y_inds[i])

        xbins.append(x_inds[-1])
        ybins.append(y_inds[-1])

        return xbins, ybins

    def dv_table(self, node):
        """
        This protocol prepares a distance-vector routing table for any node on the map.
        The DV protocol here assigns weights to map edges by calculating the sum of the distances a node is
        from the next three nodes in the original route.

        :param      node:
        :return dv_table:
        """
        possible_directions = np.array([dot for dot in G[node].__iter__()])
        nodes_already_in_route = [np.where(route_node == possible_directions)[0][0] for route_node in self.route
                                  if np.where(route_node == possible_directions)[0].size > 0]
        possible_directions = np.delete(possible_directions, nodes_already_in_route)

        reroute_node_index = np.where(node == self.route)[0][0]
        sum_three_node_dist = []
        directions = []
        for direction in possible_directions:
            twice_out = np.array([dot for dot in G[direction].__iter__()])
            if twice_out.size == 0 or (direction == self.route).any():
                # avoid culdesacs and nodes already in the route
                continue

            directions.append(direction)
            distances = []
            for compare_node in self.route[reroute_node_index + 2:reroute_node_index + 5]:
                compare_node_pos = get_position_of_node(compare_node)
                potential_node_pos = get_position_of_node(direction)
                distances.append(np.linalg.norm(compare_node_pos - potential_node_pos))
            sum_three_node_dist.append(sum(distances))

        dv_table = models.make_table({'potential-nodes': directions, 'sum-distances': sum_three_node_dist})
        return dv_table


def car_obstacles(frontview, cars):
    """
    Determines if there are any other_cars within the car's bin and then

    Parameters
    __________
    :param frontview:    object: FrontView object
    :param      cars: dataframe:

    Returns
    _______
    :return distance: list: double or False (returns False if no car obstacle found)
    """
    x_space, y_space = models.upcoming_linspace(frontview)
    if x_space.any() and y_space.any():
        other_cars = cars.drop(frontview.car.name)
        obstacles = (frontview.car['xbin'] == other_cars['xbin']) & (frontview.car['ybin'] == other_cars['ybin'])
        if obstacles.any():
            nearby_cars = other_cars[obstacles]
            for car in nearby_cars.iterrows():
                car_within_xlinspace = np.isclose(x_space, car[1]['x'], rtol=1.0e-6).any()
                car_within_ylinspace = np.isclose(y_space, car[1]['y'], rtol=1.0e-6).any()

                if car_within_xlinspace and car_within_ylinspace:
                    first_x, first_y = car[1]['x'], car[1]['y']
                    vector = (first_x - frontview.car['x'], first_y - frontview.car['y'])
                    distance = models.magnitude(vector)
                    return distance
                else:
                    return False
        else:
            return False
    else:
        return False


def light_obstacles(frontview, lights):
    """
    Determines the distance to red traffic lights. If light is green, returns False

    Parameters
    __________
    :param  frontview:    object: FrontView object
    :param     lights: dataframe:

    Returns
    _______
    :return distance: list: double for False (returns False if no red light is found)
    """
    x_space, y_space = models.upcoming_linspace(frontview)
    if x_space.any() and y_space.any():
        obstacles = (frontview.car['xbin'] == lights['xbin']) & (frontview.car['ybin'] == lights['ybin'])
        if obstacles.any():
            nearby_lights = lights[obstacles]
            for light in nearby_lights.iterrows():
                light_within_xlinspace = np.isclose(x_space[1:], light[1]['x'], rtol=1.0e-6).any()
                light_within_ylinspace = np.isclose(y_space[1:], light[1]['y'], rtol=1.0e-6).any()

                if light_within_xlinspace and light_within_ylinspace:
                    car_vector = [light[1]['x'] - frontview.car['x'], light[1]['y'] - frontview.car['y']]
                    face_values = light[1]['go-values']
                    face_vectors = [(light[1]['out-xvectors'][i], light[1]['out-yvectors'][i])
                                    for i in range(light[1]['degree'])]

                    for value, vector in zip(face_values, face_vectors):
                        if not value and models.determine_anti_parallel_vectors(car_vector, vector):
                            distance = models.magnitude(car_vector)
                            return distance
                        else:
                            continue
                else:
                    return False
        else:
            return False
    else:
        return False


def determine_pedigree(node_id):
    """
     each traffic light has a list of vectors, pointing in the direction of the road a light color should influence

     :param  node_id:    int
     :return vectors:   list: list of vectors pointing from the intersection to the nearest point on the out roads
     """
    x, y = get_position_of_node(node_id)

    out_nodes = [dot for dot in G[node_id].__iter__()]

    vectors = []
    for node in out_nodes:
        try:
            out_x, out_y = lines_to_node(node_id, node)[0][1]
        except IndexError:
            continue
        vectors.append((out_x - x, out_y - y))

    return vectors


def find_culdesacs():
    """
    culdesacs are nodes with only one edge connection and which are not on the boundary of the OpenStreetMap

    :return culdesacs: list of node IDs
    """
    culdesacs = [key for key, value in G.graph['streets_per_node'].items() if value == 1]
    return culdesacs


def find_traffic_lights(prescale=10):
    """
    traffic lights are nodes in the graph which have degree > 3

    :return light_intersections: a list of node IDs suitable for traffic lights
    """
    light_intersections = []
    for i, node in enumerate(G.degree()):
        if (node[1] > 3) and not (i % prescale):
            light_intersections.append(node)

    return light_intersections


def find_nodes(n):
    """
    returns n node IDs from the networkx graph

    :param      n: int
    :return nodes: list
    """
    nodes = []
    for node in G.nodes():
        nodes.append(node)
    return nodes[:n]


def get_position_of_node(node):
    """
    Get latitude and longitude given node ID

    :param node:      graphml node ID
    :return position: array:    [latitude, longitude]
    """
    # note that the x and y coordinates of the G.nodes are flipped
    # this is possibly an issue with the omnx G.load_graphml method
    # a correction is to make the position tuple be (y, x) as below
    position = np.array([G.nodes[node]['x'], G.nodes[node]['y']])
    return position


def get_init_path(origin, destination):
    """
    compiles a list of tuples which represents a route

    Parameters
    __________
    :param      origin: int:    node ID
    :param destination: int:    node ID

    Returns
    _______
    :return path: list where each entry is a tuple of tuples
    """
    lines = shortest_path_lines_nx(origin, destination)
    path = models.path_decompiler(lines)
    return path


def get_route(origin, destination):
    """
    acquires the typical node-based route list from NetworkX with weight=length

    :param      origin: node ID
    :param destination: node ID
    :return:     route: list of intersection nodes
    """
    return nx.shortest_path(G, origin, destination, weight='length')


def eta(car, lights, speed_limit=250):
    """
    calculates the ETA by considering traffic lights, car traffic (in future versions), and distances

    :param            car: Series
    :param         lights: DataFrame
    :param    speed_limit: int
    :return:    path_time: double
    """
    route = np.array(car['route'])

    if route.size > 0:
        route_length = sum([G.get_edge_data(route[i], route[i + 1])[0]['length'] for i in range(len(route) - 1)])

        eta_from_distance = route_length / speed_limit

        light_locs = [(node == lights['node']).tolist().index(True) for node in route if (node == lights['node']).any()]

        # let the expected wait time for all lights found in the route be half the sum of the times
        expected_wait = sum([lights.loc[index]['switch-time'] for index in light_locs]) / 2
        path_time = eta_from_distance + expected_wait
    else:
        path_time = 0
    return path_time


# TODO: bundle this method into StateView, and use dv_table; by first making StateView.dv_table method more abstract
def build_new_route(route, reroute_node, direction, traffic, avoid):
    """
    this function builds a new route for a car based on the original route given that it would like to turn off
    the original route at the reroute_node

    :param        route: array: the original Dijkstra's shortest path
    :param reroute_node:   int: the node at which the car would like to depart the original path
    :param    direction:   int: the next node after reroute_node in the direction of the departure
    :param      traffic:   int: 0 or n (0 if new route avoids a traffic light, n if new route avoids n-node traffic)
    :param        avoid:   int: the node on which avoidance is based

    :return:  new_route, x_path, y_path, detour: lists: the new route, along with its x and y lines, and the detour path
    """
    reroute_index = np.where(route == reroute_node)[0][0]
    avoid_index = np.where(route == avoid)[0][0]
    new_route = route[:reroute_index + 1].tolist()
    new_route.append(direction)
    detour = [reroute_node, direction]

    # get the coordinate positions of the next three nodes in the original route
    # TODO: this will not work if we are building a new route near the very end of a route, where there are not 3 nodes
    next_nodes_pos = []
    for node in route[reroute_index + 1:reroute_index + 4]:
        x, y = get_position_of_node(node)
        next_nodes_pos.append((x, y))

    returned = False
    i = 0
    while not returned:
        i += 1
        if i == 10:
            print('Could not build new route for route {} with avoid_node={}. 10th walk was at node {}'.format(
                route, avoid, direction
            ))
            break
        out_from_direction = [dot for dot in G[direction].__iter__() if dot != reroute_node]

        # Populate a list of the sums of the distances to the next
        # three nodes in the original route, for each potential new node
        sum_three_node_dist, refined_out_from_direction = [], []
        for node in out_from_direction:
            if (node == route[:avoid_index + 1 + traffic]).any():
                # avoid all the nodes in the route including the ones around which we are rerouting
                continue

            twice_out = np.array([dot for dot in G[node].__iter__()])
            if (direction == twice_out).any():
                twice_out = np.delete(twice_out, np.where(twice_out == direction)[0][0])

            if twice_out.size == 0:
                # avoid culdesacs
                continue

            distances = []
            for compare_node in next_nodes_pos:
                potential_node_pos = get_position_of_node(node)
                distances.append(np.linalg.norm(compare_node - potential_node_pos))
            sum_three_node_dist.append(sum(distances))
            refined_out_from_direction.append(node)

        sum_three_node_dist = np.array(sum_three_node_dist)
        refined_out_from_direction = np.array(refined_out_from_direction)

        if refined_out_from_direction.size == 0:
            # unable to continue rerouting, try rerouting from an earlier node in the route
            return False

        next_node = refined_out_from_direction[sum_three_node_dist.argsort()[0]]

        if (next_node == route[:reroute_index]).any():
            # going in circles, try rerouting from an earlier node in the route
            return False

        new_route.append(next_node)
        detour.append(next_node)
        if (next_node == route[reroute_index + 1 + traffic:]).any():
            start_at_index = np.where(route == next_node)[0][0]
            for node in route[start_at_index + 1:]:
                new_route.append(node)
            returned = True
        else:
            reroute_node = direction
            direction = next_node

    lines = []
    for i in range(len(new_route)):
        if i < len(new_route) - 1:
            lines.append(shortest_path_lines_nx(new_route[i], new_route[i + 1]))

    new_path = []
    for geometry in lines:
        for point in geometry[0]:
            new_path.append(point)

    new_clean_path = models.new_route_decompiler(new_path)
    new_xpath, new_ypath = [point[0] for point in new_clean_path], [point[1] for point in new_clean_path]
    return new_route, new_xpath, new_ypath, detour


def lines_to_node(origin, destination):
    """
    return the points of all nodes in the route, including the minor nodes which make up line geometry

    :param      origin: int
    :param destination: int
    :return      lines: list
    """

    route = nx.shortest_path(G, origin, destination, weight='length')

    # find the route lines
    edge_nodes = list(zip(route[:-1], route[1:]))
    lines = []
    for u, v in edge_nodes:
        # if there are parallel edges, select the shortest in length
        data = min(G.get_edge_data(u, v).values(), key=lambda x: x['length'])

        # if it has a geometry attribute (ie, a list of line segments)
        if 'geometry' in data:
            # add them to the list of lines to plot
            xs, ys = data['geometry'].xy
            lines.append(list(zip(xs, ys)))
        else:
            # if it doesn't have a geometry attribute, the edge is a straight
            # line from node to node
            x1 = G.nodes[u]['x']
            y1 = G.nodes[u]['y']
            x2 = G.nodes[v]['x']
            y2 = G.nodes[v]['y']
            line = ((x1, y1), (x2, y2))
            lines.append(line)

    return lines


def shortest_path_lines_nx(origin, destination):
    """
    uses the default shortest path algorithm available through networkx

    Parameters
    __________
    :param      origin: int:    node ID
    :param destination: int:    node ID

    Returns
    _______
    :return lines: list:
        [(double, double), ...]:   each tuple represents the bend-point in a straight road
    """

    route = nx.shortest_path(G, origin, destination, weight='length')

    # find the route lines
    edge_nodes = list(zip(route[:-1], route[1:]))
    lines = []
    for u, v in edge_nodes:
        # if there are parallel edges, select the shortest in length
        data = min(G.get_edge_data(u, v).values(), key=lambda x: x['length'])

        # if it has a geometry attribute (ie, a list of line segments)
        if 'geometry' in data:
            # add them to the list of lines to plot
            xs, ys = data['geometry'].xy
            lines.append(list(zip(xs, ys)))
        else:
            # if it doesn't have a geometry attribute, the edge is a straight
            # line from node to node
            x1 = G.nodes[u]['x']
            y1 = G.nodes[u]['y']
            x2 = G.nodes[v]['x']
            y2 = G.nodes[v]['y']
            line = ((x1, y1), (x2, y2))
            lines.append(line)

    return lines
