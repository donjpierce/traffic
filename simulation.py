"""
Description of module...
"""
import osmnx as ox
import networkx as nx
import pandas as pd

import navigation as nav


# fill the initial state with N cars
N = 1
dt = 1/1000

TEMP_destination_node = 53082632


def init_culdesac_start_location(N):
    """
    initializes N cars into N culdesacs

    Parameters
    __________
    :param     N:   int

    Returns
    _______
    :return cars:   array:  [dict, ...]
    """
    culdesacs = [key for key, value in G.graph['streets_per_node'].items() if value == 1]
    cars = []

    for i in range(N):
        start_node = culdesacs[i]
        position = nav.get_position(start_node)

        cars.append(
            {'position': position,
             'velocity': [0, 0],
             'acceleration': [0, 0],
             'front-view': 0,
             'destination': TEMP_destination_node
             }
        )

    return cars


cars_dict = init_culdesac_start_location(N)
cars_df = pd.DataFrame(cars_dict)
cars_state = cars.State(cars_df)
