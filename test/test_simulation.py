import pytest
import tqdm

import cars
import simulation


def test_car_simulation_works(test_ograph):
    test_ograph.project_axis()
    init_cars = simulation.init_random_node_start_location(100, test_ograph)
    cars_object = cars.Cars(init_state=init_cars, graph=test_ograph)

    # initialize lights
    init_lights = simulation.init_traffic_lights(test_ograph, prescale=40)
    lights_object = cars.TrafficLights(light_state=init_lights, graph=test_ograph)

    cars_data, lights_data = [], []
    for _ in tqdm.tqdm(range(10)):
        cars_data.append(cars_object.update(dt=0.1, lights=lights_object.state))
        lights_data.append(lights_object.update(dt=0.1))
    data = {"cars": cars_data, "lights": lights_data}
    assert len(data["cars"]) == 10
