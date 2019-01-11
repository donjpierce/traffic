# python=3.6 requires using Qt4Agg backend for animation saving
import matplotlib
matplotlib.use('Qt4Agg')
from cars import Cars, TrafficLights
import convergent_learner as cl
from matplotlib import animation
import osmnx as ox
import simulation as sim


class Animator:
    def __init__(self, cars_object, lights_object, dt=1 / 1000, N=1):
        self.dt = dt
        self.N = N
        self.cars_object = cars_object
        self.lights_object = lights_object
        self.number_of_lights = len(self.lights_object.state)
        self.number_of_faces = sum(self.lights_object.state['degree'])
        self.cars = sum([ax.plot([], [], color='blue', marker='o', ms=3) for n in range(self.N)], [])
        self.lights = sum([ax.plot([], [], color='red', marker='+', ms=2) for l in range(self.number_of_lights)], [])
        self.faces = sum([ax.plot([], [], color='red', marker='^', ms=2) for f in range(self.number_of_faces)], [])

    def reset(self):
        """
        Set initial blank data

        :return cars + lights + faces:
        """
        for car in self.cars:
            car.set_data([], [])

        for light in self.lights:
            light.set_data([], [])

        for face in self.faces:
            face.set_data([], [])

        return self.cars + self.lights + self.faces

    def animate(self, i):
        """
        perform one animation step

        :param i: animation step
        :return:
        """
        self.lights_object.update(self.dt)
        self.cars_object.update(self.dt, self.lights_object.state)

        for car, car_series in zip(self.cars, self.cars_object.state.iterrows()):
            x = car_series[1]['x']
            y = car_series[1]['y']
            car.set_data(x, y)

        face_positions = []
        face_colors = []

        for light, light_series in zip(self.lights, self.lights_object.state.iterrows()):
            xs = light_series[1]['out-xpositions']
            ys = light_series[1]['out-ypositions']
            face_go_values = light_series[1]['go-values']

            light.set_data(light_series[1]['x'], light_series[1]['y'])

            for coords in zip(xs, ys):
                face_positions.append(coords)

            for color in face_go_values:
                face_colors.append(color)

        for face, position, color in zip(self.faces, face_positions, face_colors):
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
        # ax.set_ylim(4186020, 4186300)

        # limits for viewing special area for machine learning tests
        # ax.set_xlim(567295, 568600)
        # ax.set_ylim(4186360, 4187450)

        fig.canvas.draw()
        return self.cars + self.lights + self.faces


# load figure for animation
"""Lower Manhattan"""
# G = ox.load_graphml('lowermanhattan.graphml')
# G = ox.project_graph(G)
# fig, ax = ox.plot_graph(G, fig_height=12, node_size=0, edge_linewidth=0.5)
# ax.set_title('Lower Manhattan, New York City')


"""San Francisco"""
# G = ox.load_graphml('sanfrancisco.graphml')
# G = ox.project_graph(G)
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
# cars_object = Cars(sim.init_culdesac_start_location(N, axis), axis)
# cars_object = Cars(sim.init_random_node_start_location(N, axis), axis)
# lights_object = TrafficLights(sim.init_traffic_lights(axis, prescale=40), axis)
cars = Cars(cl.init_custom_agent(n=1, fig_axis=axis), axis=axis)
lights = TrafficLights(cl.init_custom_lights(fig_axis=axis, prescale=None), axis)


# initialize the Animator
animator = Animator(cars_object=cars, lights_object=lights)
animate = animator.animate
init = animator.reset

# for creating HTML frame-movies
# ani = animation.FuncAnimation(fig, animate, init_func=init, frames=1200, interval=30, blit=True)
# ani.save('traffic.html', fps=300, extra_args=['-vcodec', 'libx264'])

# for creating mp4 movies
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=20000)
mywriter = animation.FFMpegWriter(fps=300)
ani.save('movie.mp4', writer=mywriter)
