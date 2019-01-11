class Animator:
    def __init__(self, fig, ax, cars_object, lights_object, num, dt=1 / 1000, n=1):
        self.fig = fig
        self.ax = ax
        self.num = num
        self.dt = dt
        self.N = n
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

        # limits for the path view of 1 car with TEMP_dest_node destination
        # ax.set_xlim(566730, 567270)
        # ax.set_ylim(4185840, 4186260)

        # limits for viewing 1st traffic light in Piedmont
        # ax.set_xlim(566930, 567404)
        # ax.set_ylim(4186020, 4186300)

        # limits for viewing special area for machine learning tests
        # ax.set_xlim(567295, 568600)
        # ax.set_ylim(4186360, 4187450)

        # limits for convergent_learner.py
        self.ax.set_xlim(567012, 567809)
        self.ax.set_ylim(4186330, 4187070)

        axis = self.ax.axis()
        self.ax.annotate('Episode {} of {}'.format(self.num[0], self.num[1]), xy=(axis[0] + 10, axis[2] + 10))

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

        self.fig.canvas.draw()
        try:
            self.fig.savefig('frames/frame{}'.format(i))
        except FileNotFoundError:
            raise Exception('Please make a folder called "frames" in this project directory')

        return self.cars + self.lights + self.faces
