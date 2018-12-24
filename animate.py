from cars import Cars, TrafficLights
from matplotlib import animation
import numpy as np
import osmnx as ox
import simulation as sim

dt = 1 / 1000
N = 33

# load figure for animation
"""Lower Manhattan"""
# G = ox.load_graphml('lowermanhattan.graphml')
# G = ox.project_graph(G)
# fig, ax = ox.plot_graph(G, fig_height=12, node_size=0, edge_linewidth=0.5)
# ax.set_title('Lower Manhattan, New York City')

"""San Francisco"""
# G = ox.load_graphml('sanfrancisco.graphml')
# fig, ax = ox.plot_graph(G, fig_height=12, fig_width=10, node_size=0, edge_linewidth=0.5)
# ax.set_title('San Francisco, California')

"""Piedmont, California"""
G = ox.load_graphml('piedmont.graphml')
G = ox.project_graph(G)
fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5)
ax.set_title('Piedmont, California')


# grab / set information about the figure and axes
axis_range = ax.axis()
xy_range = (axis_range[1] - axis_range[0], axis_range[3] - axis_range[2])
# ax.set_xlim(566730, 567270)
# ax.set_ylim(4185840, 4186260)


# initialize traffic lights
number_of_lights = len(sim.init_traffic_lights())
number_of_faces = sum([sim.init_traffic_lights()['degree'][i] for i in range(number_of_lights)])


# initialize empty points for animation
cars = sum([ax.plot([], [], color='blue', marker='o', ms=3) for n in np.arange(N)], [])
lights = sum([ax.plot([], [], color='red', marker='+', ms=2) for l in np.arange(number_of_lights)], [])
faces = sum([ax.plot([], [], color='red', marker='^', ms=2) for f in np.arange(number_of_faces)], [])


# initialize the car and light state objects
cars_state = Cars(sim.init_culdesac_start_location(N))
lights_state = TrafficLights(sim.init_traffic_lights())


def init():
    """
    initializes the animation
    :return: particles, cube
    """
    for car in cars:
        car.set_data([], [])

    for light in lights:
        light.set_data([], [])

    for face in faces:
        face.set_data([], [])

    return cars + lights + faces


def animate(i):
    """
    perform animation step
    :param i:
    :return:
    """
    lights_state.update(dt)
    cars_state.update(dt, lights_state, xy_range)

    for car, car_dict in zip(cars, cars_state.state):
        x = car_dict['position'][0]
        y = car_dict['position'][1]
        car.set_data(x, y)

    face_positions = []
    face_colors = []

    for light, light_dict in zip(lights, lights_state.state):
        xs = [light_dict['out-positions'][i][0] for i in range(light_dict['degree'])]
        ys = [light_dict['out-positions'][i][1] for i in range(light_dict['degree'])]
        face_go_values = [light_dict['pedigree'][i]['go'] for i in range(light_dict['degree'])]

        x, y = light_dict['position']
        light.set_data(x, y)

        for coords in zip(xs, ys):
            face_positions.append(coords)

        for color in face_go_values:
            face_colors.append(color)

    for face, position, color in zip(faces, face_positions, face_colors):
        face.set_data(position[0], position[1])
        if color:
            face.set_color('green')
        else:
            face.set_color('red')

    # limits for the path view of 1 car with TEMP_dest_node destination
    # ax.set_xlim(566730, 567270)
    # ax.set_ylim(4185840, 4186260)

    # limits for viewing 1st traffic light in Piedmont
    # ax.set_xlim(566930, 567404)
    # ax.set_ylim(4186020, 4186280)

    fig.canvas.draw()
    return cars + lights + faces


# for creating HTML frame-movies
# ani = animation.FuncAnimation(fig, animate, init_func=init, frames=1200, interval=30, blit=True)
# ani.save('traffic.html', fps=300, extra_args=['-vcodec', 'libx264'])

# for creating movies
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=50000)
mywriter = animation.FFMpegWriter(fps=300)
ani.save('movie.mp4', writer=mywriter)
