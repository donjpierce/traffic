from cars import Cars, TrafficLights
from matplotlib import animation
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


# grab the dimensions of the figure
axis = ax.axis()


# initialize the car and light state objects
cars_object = Cars(sim.init_culdesac_start_location(N, axis), axis)
# cars_object = Cars(sim.init_random_node_start_location(N, axis), axis)
lights_object = TrafficLights(sim.init_traffic_lights(axis), axis)


# initialize traffic lights
number_of_lights = len(lights_object.state)
number_of_faces = sum(lights_object.state['degree'])


# initialize empty points for animation
cars = sum([ax.plot([], [], color='blue', marker='o', ms=3) for n in range(N)], [])
lights = sum([ax.plot([], [], color='red', marker='+', ms=2) for l in range(number_of_lights)], [])
faces = sum([ax.plot([], [], color='red', marker='^', ms=2) for f in range(number_of_faces)], [])


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
    lights_object.update(dt)
    cars_object.update(dt, lights_object.state)

    for car, car_series in zip(cars, cars_object.state.iterrows()):
        x = car_series[1]['x']
        y = car_series[1]['y']
        car.set_data(x, y)

    face_positions = []
    face_colors = []

    for light, light_series in zip(lights, lights_object.state.iterrows()):
        xs = light_series[1]['out-xpositions']
        ys = light_series[1]['out-ypositions']
        face_go_values = light_series[1]['go-values']

        light.set_data(light_series[1]['x'], light_series[1]['y'])

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
    ax.set_xlim(566930, 567404)
    ax.set_ylim(4186020, 4186300)

    # limits for viewing special area for machine learning tests
    # ax.set_xlim(566960, 567701)
    # ax.set_ylim(4186510, 4186940)

    fig.canvas.draw()
    return cars + lights + faces


# for creating HTML frame-movies
# ani = animation.FuncAnimation(fig, animate, init_func=init, frames=1200, interval=30, blit=True)
# ani.save('traffic.html', fps=300, extra_args=['-vcodec', 'libx264'])

# for creating movies
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=50000)
mywriter = animation.FFMpegWriter(fps=300)
ani.save('movie.mp4', writer=mywriter)
