from cars import Cars, TrafficLights
from matplotlib import animation
import models
import numpy as np
import osmnx as ox
import simulation as sim

dt = 1 / 1000
N = 33

# load figure for animation
G = ox.load_graphml('piedmont.graphml')
G = ox.project_graph(G)
fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5)
ax.set_title('Piedmont, California')

# G = ox.load_graphml('sanfrancisco.graphml')
# fig, ax = ox.plot_graph(G, fig_height=12, fig_width=10, node_size=0, edge_linewidth=0.5)
# ax.set_title('San Francisco, California')

# initialize empty particle points for animation
cars_state = Cars(sim.init_culdesac_start_location(N))
cars = sum([ax.plot([], [], color='brown', marker='o', ms=3) for n in np.arange(N)], [])
# state = Cars(sim.init_random_node_start_location(N))


# initialize traffic lights
number_of_lights = len(sim.init_traffic_lights())
initial_colors = models.initial_light_colors(number_of_lights)
lights_state = TrafficLights(sim.init_traffic_lights())
lights = sum([ax.plot([], [], color=initial_colors[l], marker='o', ms=2) for l in np.arange(number_of_lights)], [])


def init():
    """
    initializes the animation
    :return: particles, cube
    """
    for car in cars:
        car.set_data([], [])

    for light in lights:
        light.set_data([], [])

    return cars + lights


def animate(i):
    """
    perform animation step
    :param i:
    :return:
    """

    cars_state.update(dt)
    lights_state.update(dt)

    for car, car_dict in zip(cars, cars_state.state):
        x = car_dict['position'][0]
        y = car_dict['position'][1]
        car.set_data(x, y)

    for light, light_dict in zip(lights, lights_state.state):
        x = light_dict['position'][0]
        y = light_dict['position'][1]
        go = light_dict['go'][0]
        light.set_data(x, y)
        if go:
            light.set_color('green')
        else:
            light.set_color('red')

    fig.canvas.draw()
    return cars + lights


# for creating HTML frame-movies
# ani = animation.FuncAnimation(fig, animate, init_func=init, frames=1200, interval=30, blit=True)
# ani.save('traffic.html', fps=300, extra_args=['-vcodec', 'libx264'])

# for creating movies
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=11000)
mywriter = animation.FFMpegWriter(fps=300)
ani.save('movie.mp4', writer=mywriter)
