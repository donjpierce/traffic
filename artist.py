# python=3.6 requires using Qt4Agg backend for animation saving
import os
from animate import Animator
from datetime import datetime as dt
import matplotlib
# matplotlib.use('Qt4Agg')
from cars import Cars, TrafficLights
# import convergent_learner as cl
from matplotlib import animation
import osmnx as ox
import simulation as sim
from osm_request import OGraph

# ask the user for a geo-codable location
query = input('Please input a geo-codable place, like "Harlem, NY" or "Kigali, Rwanda": ')

# get OGraph object
graph = OGraph(query, save=True)


# initialize the car and light state objects
N = 300  # number of cars to simulate
# cars = Cars(sim.init_culdesac_start_location(N, axis), axis)
cars = Cars(sim.init_random_node_start_location(N, graph), graph)
lights = TrafficLights(sim.init_traffic_lights(graph, prescale=15), graph)

""" for an example of learning using a single, convergent learner, initialize the sim using these cars and lights: """
# cars = Cars(cl.init_custom_agent(n=1, fig_axis=axis), axis=axis)
# lights = TrafficLights(cl.init_custom_lights(fig_axis=axis, prescale=None), axis)

# time of simulation (in seconds)
duration = 30
frames_per_second = 60
n_frames = duration * frames_per_second

# initialize the Animator
animator = Animator(fig=graph.fig, ax=graph.ax, cars_object=cars, lights_object=lights, num=(1, 1), n=N)
init = animator.reset
animate = animator.animate

print(f"{dt.now().strftime('%H:%M:%S')} Now running simulation... ")
# for creating HTML movies
ani = animation.FuncAnimation(graph.fig, animate, init_func=init, frames=n_frames, interval=30, blit=True)
mywriter = animation.HTMLWriter(fps=frames_per_second)
ani.save('traffic.html', writer=mywriter)

# for creating mp4 movies
# ani = animation.FuncAnimation(fig, animate, init_func=init, frames=500)
# mywriter = animation.FFMpegWriter(fps=60)
# ani.save('movie.mp4', writer=mywriter)
