import cProfile
import pstats

from cars import Cars, TrafficLights
import simulation as sim
from osm_request import OGraph
import tqdm

# Test graph
graph = OGraph("Olney, Maryland")


def test_cars_update(timesteps=100):
    # initialize cars
    init_cars = sim.init_random_node_start_location(100, graph)
    cars_object = Cars(init_state=init_cars, graph=graph)

    # initialize lights
    init_lights = sim.init_traffic_lights(graph, prescale=40)
    lights_object = TrafficLights(light_state=init_lights, graph=graph)

    # update
    cars_data, lights_data = [], []
    for i in tqdm.tqdm(range(timesteps)):
        cars_data.append(
            cars_object.update(dt=0.1, lights=lights_object.state)
        )
        lights_data.append(
            lights_object.update(dt=0.1)
        )
    data = {'cars': cars_data, 'lights': lights_data}
    return data


if __name__ == '__main__':

    profiler = cProfile.Profile()
    profiler.enable()

    # begin test function call
    test_cars_update()
    # end tests

    profiler.disable()
    profiler.dump_stats('profile_cars_update.prof')
    p = pstats.Stats('profile_cars_update.prof')
    p.sort_stats('cumtime').print_stats(100)

