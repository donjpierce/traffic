# python=3.6 requires using Qt4Agg backend for animation saving
from animate import Animator
import matplotlib
# matplotlib.use('Qt4Agg')
from cars import Cars, TrafficLights
# import convergent_learner as cl
from matplotlib import animation
import osmnx as ox
import simulation as sim


# load figure for animation
""" Manhattan """
# G = ox.load_graphml('manhattan.graphml')
# G = ox.project_graph(G)
# fig, ax = ox.plot_graph(G, fig_height=30, node_size=0, edge_linewidth=0.5)
# ax.set_title('Manhattan, New York City')

"""Lower Manhattan"""
# G = ox.load_graphml('data/lowermanhattan.graphml')
# G = ox.project_graph(G)
# fig, ax = ox.plot_graph(G, fig_height=12, node_size=0, edge_linewidth=0.5)
# ax.set_title('Lower Manhattan, New York City')


"""San Francisco"""
# G = ox.load_graphml('data/sanfrancisco.graphml')
# G = ox.project_graph(G)
# fig, ax = ox.plot_graph(G, fig_height=12, fig_width=10, node_size=0, edge_linewidth=0.5)
# ax.set_title('San Francisco, California')


"""Piedmont, California"""
G = ox.load_graphml('piedmont.graphml')
G = ox.project_graph(G)
fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5, show=False)
ax.set_title('Piedmont, California')


# grab the dimensions of the figure
axis = ax.axis()


""" initialize the car and light state objects """
N = 80  # cars
# cars = Cars(sim.init_culdesac_start_location(N, axis), axis)
cars = Cars(sim.init_culdesac_start_location(N, axis), axis)
lights = TrafficLights(sim.init_traffic_lights(axis, prescale=100), axis)

""" for an example of learning using a single, convergent learner, initialize the sim using these cars and lights: """
# cars = Cars(cl.init_custom_agent(n=1, fig_axis=axis), axis=axis)
# lights = TrafficLights(cl.init_custom_lights(fig_axis=axis, prescale=None), axis)


# initialize the Animator
animator = Animator(fig=fig, ax=ax, cars_object=cars, lights_object=lights, num=(1, 10), n=N)
init = animator.reset
animate = animator.animate

# for creating HTML movies
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=500, interval=30, blit=True)
mywriter = animation.HTMLWriter(fps=60)
ani.save('traffic.html', writer=mywriter)

# for creating mp4 movies
# ani = animation.FuncAnimation(fig, animate, init_func=init, frames=500)
# mywriter = animation.FFMpegWriter(fps=60)
# ani.save('movie.mp4', writer=mywriter)
