"""
Usage:

python artist.py --location "Harlem, NY" --cars 10 --duration 60 --fps 30 --interactive \
 --light_prescaling 1 --learning --mp4 --serialize

--location: Must be a geocode-able location, like "Washington, DC, USA" or "Campo Limpo, São Paulo, Brazil"
--cars: Number of cars to simulate
--duration: Duration of simulation (in seconds)
--fps: Frames per second
--interactive: If True, will prompt the user for simulation parameters not specified in the command line
--light_prescaling: All intersections of degree 3 or higher on the map will have a traffic light if not pre-scaled
                    by this parameter. The large the map the smaller this param should be.
                        2 -> 1/2 qualifying intersections will not have a light (good for large cities)
                        10 -> 1/10 qualifying intersections will not have a light (good for metropolitan areas)
                        100 -> 1/100 qualifying intersections will not have a light (good for rural areas)

"""

import argparse
from animate import Animator
from datetime import datetime as dt
from cars import Cars, TrafficLights
import convergent_learner as cl
from matplotlib import animation
import osmnx as ox
import simulation as sim
from osm_request import OGraph
from tqdm import tqdm
import sys


# Listen for CLI args
parser = argparse.ArgumentParser(
    prog='Artist',
    description='A module to generate HTML or MP4 movies of a traffic simulation.'
)

parser.add_argument('-l', '--location', type=str, help='A geocode-able location over which to simulate traffic.')
parser.add_argument('-c', '--cars', type=int, help='The number of cars to simulate.')
parser.add_argument('-d', '--duration', type=int, help='The duration of the simulation (in seconds).')
parser.add_argument('-f', '--frames_per_second', type=int, help='The number of frames per second to render.')
parser.add_argument('-i', '--interactive', action='store_true', help='Run the simulation in interactive mode.')
parser.add_argument('-p', '--light_prescaling', type=int, help='The number of lights to prescale.')
parser.add_argument('-x', '--learning', action='store_true', help='Run the simulation in learning mode.')
parser.add_argument('-m', '--mp4', action='store_true', help='Generate an MP4 movie instead of an HTML movie.')
parser.add_argument('-s', '--serialize', action='store_true', help='Serialize the simulation in parquet dataframes.')


def main(
        location,
        cars,
        duration,
        frames_per_second,
        interactive,
        light_prescaling,
        learning,
        mp4,
        serialize
):
    """"

    :param location: str
    :param cars: int
    :param duration: int
    :param frames_per_second: int
    :param interactive: bool
    :param light_prescaling: int
    :param learning: bool
    :param mp4: bool
    :param serialize: bool
    """
    query, N = location, cars

    # enable interactive mode
    if interactive:
        # ask the user for simulation parameters, or retain them if already specified
        query = location or input('Please input a geo-codable place, like "Harlem, NY" or "Kigali, Rwanda": ')
        N = cars or int(input('Number of cars to simulate: '))
        # time of simulation (in seconds)
        duration = int(input('Duration of time to simulate (in seconds): '))

    # get OGraph object)
    graph = OGraph(query, save=True)

    # get the simulation methods
    # if a learning agent should be used, use the convergent learner init methods
    if not learning:
        # Default mode:
        # initialize the car and light state objects
        # cars = Cars(sim.init_culdesac_start_location(N, graph), graph)  # TODO: parametrize
        cars = Cars(sim.init_random_node_start_location(N, graph), graph, serialize=serialize)
        lights = TrafficLights(sim.init_traffic_lights(graph, prescale=light_prescaling), graph=graph)
    else:
        fig, axis = ox.plot_graph(graph.G, node_size=0, edge_linewidth=0.5)
        cars = Cars(cl.init_custom_agent(graph, n=1), graph)
        lights = TrafficLights(cl.init_custom_lights(fig_axis=axis, prescale=None), axis)

    # calculate the number of frames to simulate
    n_frames = duration * frames_per_second

    # initialize the Animator
    animator = Animator(fig=graph.fig, ax=graph.ax, cars_object=cars, lights_object=lights, num=(1, 1), n=N)
    init = animator.reset
    animate = animator.animate

    print(f"{dt.now().strftime('%H:%M:%S')} Now running simulation... ")
    if not mp4:
        # for creating HTML movies
        ani = animation.FuncAnimation(graph.fig, animate,
                                      init_func=init,
                                      frames=tqdm(range(n_frames), file=sys.stdout),
                                      interval=duration,
                                      blit=True)
        mywriter = animation.HTMLWriter(fps=frames_per_second)
        ani.save(f'traffic_{dt.today().strftime("%Y_%m_%d")}.html', writer=mywriter)
    else:
        # for creating mp4 movies
        ani = animation.FuncAnimation(graph.fig, animate, init_func=init, frames=n_frames)
        mywriter = animation.FFMpegWriter(fps=frames_per_second)
        ani.save(f'traffic_{dt.today().strftime("%Y_%m_%d")}.mp4', writer=mywriter)

    return


if __name__ == '__main__':

    args = parser.parse_args()
    default_args = {
        'location': "Olney, Maryland",
        'cars': 50,
        'duration': 10,
        'frames_per_second': 60,
        'interactive': False,
        'light_prescaling': 15,
        'learning': False,
        'mp4': False,
        'serialize': False
    }
    provided_args = {
        key: args.__getattribute__(key) if args.__getattribute__(key) is not None else default_args[key]
        for key in vars(args)
    }
    main(**provided_args)

