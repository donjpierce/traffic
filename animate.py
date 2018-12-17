from cars import Cars
from matplotlib import animation
import numpy as np
import osmnx as ox
import simulation as sim

dt = 1 / 1000
N = 10

# load figure for animation
# G = ox.load_graphml('piedmont.graphml')
G = ox.load_graphml('sanfrancisco.graphml')
G = ox.project_graph(G)
fig, ax = ox.plot_graph(G, fig_height=12, fig_width=10, node_size=0, edge_linewidth=0.5)
ax.set_title('San Francisco, California')

# initialize empty particle points for animation
cars = sum([ax.plot([], [], 'ro', ms=3) for n in np.arange(N)], [])
# state = Cars(sim.init_culdesac_start_location(N))
state = Cars(sim.init_random_node_start_location(N))


def init():
    """
    initializes the animation
    :return: particles, cube
    """
    for car in cars:
        car.set_data([], [])
    return cars


def animate(i):
    """
    perform animation step
    :param i:
    :return:
    """

    state.update(dt)

    for car, car_dict in zip(cars, state.state):
        x = car_dict['position'][0]
        y = car_dict['position'][1]
        car.set_data(x, y)

    fig.canvas.draw()
    return cars


# for creating HTML frame-movies
# ani = animation.FuncAnimation(fig, animate, init_func=init, frames=4000, interval=1, blit=True)
# ani.save('traffic.html', fps=100, extra_args=['-vcodec', 'libx264'])

# for creating movies
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=20000)
mywriter = animation.FFMpegWriter(fps=200)
ani.save('movie.mp4', writer=mywriter)
