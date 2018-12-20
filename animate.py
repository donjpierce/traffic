from cars import Cars, TrafficLights
from matplotlib import animation
import models
import numpy as np
import osmnx as ox
import simulation as sim

dt = 1 / 1000
N = 6

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
cars = sum([ax.plot([], [], color='blue', marker='o', ms=3) for n in np.arange(N)], [])
# state = Cars(sim.init_random_node_start_location(N))

# initialize traffic lights
number_of_lights = len(sim.init_traffic_lights())
number_of_faces = sum([sim.init_traffic_lights()[i]['degree'] for i in range(number_of_lights)])
# initial_colors = models.initial_light_colors(number_of_lights)
lights_state = TrafficLights(sim.init_traffic_lights())
lights = sum([ax.plot([], [], color='red', marker='+', ms=2) for l in np.arange(number_of_lights)], [])
faces = sum([ax.plot([], [], color='red', marker='^', ms=2) for f in np.arange(number_of_faces)], [])


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

    light_conditions = [(lights_state.state[i]['position'], lights_state.state[i]['pedigree'])
                        for i in range(len(lights_state.state))]

    cars_state.update(dt, light_conditions)

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

    fig.canvas.draw()
    return cars + lights + faces


# for creating HTML frame-movies
# ani = animation.FuncAnimation(fig, animate, init_func=init, frames=1200, interval=30, blit=True)
# ani.save('traffic.html', fps=300, extra_args=['-vcodec', 'libx264'])

# for creating movies
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=20000)
mywriter = animation.FFMpegWriter(fps=300)
ani.save('movie.mp4', writer=mywriter)
