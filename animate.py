import navigation as nav


class Animator:
    def __init__(self, fig, ax, cars_object, lights_object, num,
                 frame_rate=1000, dt=1 / 1000, n=1, focus=None, write_frames=False):
        self.fig = fig
        self.ax = ax
        self.num = num
        self.frame_rate = frame_rate
        self.dt = dt
        self.N = n
        self.focus = focus  # the car ID on which the Animator should focus
        self.write_frames = write_frames  # save a PNG of the canvas every frame_rate frames
        self.cars_object = cars_object
        self.lights_object = lights_object
        self.number_of_lights = len(self.lights_object.state)
        self.number_of_faces = sum(self.lights_object.state['degree'])
        self.cars = sum([ax.plot([], [], color='blue', marker='o', ms=1) for n in range(self.N)], [])
        self.lights = sum([ax.plot([], [], color='red', marker='+', ms=2) for l in range(self.number_of_lights)], [])
        self.faces = sum([ax.plot([], [], color='red', marker='^', ms=2) for f in range(self.number_of_faces)], [])

    def reset(self, num=None):
        """
        Set initial blank data

        :num:    tuple: int, int
        :return: cars + lights + faces:
        """
        for car in self.cars:
            car.set_data([], [])

        for light in self.lights:
            light.set_data([], [])

        for face in self.faces:
            face.set_data([], [])

        if self.focus:
            # narrow the plot axes to the route of the car with ID self.focus
            route = self.cars_object.state.loc[self.focus]['route']
            new_axis = nav.determine_limits(self.cars_object.graph, route)
            self.ax.set_xlim(new_axis[0], new_axis[1])
            self.ax.set_ylim(new_axis[2], new_axis[3])
            # set the focus car to be a different color than all others
            self.cars[self.focus].set_color('green')

        axis = self.ax.axis()

        self.num = num if num else self.num
        self.ax.annotate('Episode {} of {}'.format(self.num[0], self.num[1]), xy=(axis[0] + 10, axis[2] + 10))
        self.fig.canvas.draw()

        return self.cars + self.lights + self.faces

    def animate(self, i):
        """
        perform one animation step

        :param   i:   int: animation step
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

        if i % self.frame_rate == 0 and self.write_frames:
            self.save_figure(i)

        self.fig.canvas.draw()
        return self.cars + self.lights + self.faces

    def save_figure(self, i):
        """
        saves figure as png

        :return None:
        """
        try:
            self.fig.savefig('frames/episode{}_of{}_frame{}'.format(self.num[0] + 1, self.num[1], i))
        except FileNotFoundError:
            raise Exception('Please make a folder called "frames" in this project directory')

        return None
