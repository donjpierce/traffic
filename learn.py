from environment import Env
from keras import *
import numpy as np
import osmnx as ox

dt = 1 / 1000
N = 33

"""Lower Manhattan"""
# G = ox.load_graphml('lowermanhattan.graphml')
# G = ox.project_graph(G)
# fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5)

"""San Francisco"""
# G = ox.load_graphml('sanfrancisco.graphml')
# G = ox.project_graph(G)
# fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5)

"""Piedmont, California"""
G = ox.load_graphml('piedmont.graphml')
G = ox.project_graph(G)
fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5)

# grab the dimensions of the figure
axis = ax.axis()

# initialize the environment for the learning agent
env = Env(n=N, fig_axis=axis, agent=1)

# initialize the Keras training model
model = Sequential()
model.add(layers.InputLayer(batch_input_shape=(1, 10)))
model.add(layers.Dense(10, activation='sigmoid'))
model.add(layers.Dense(2, activation='linear'))
model.compile(loss='mse', optimizer='adam', metrics=['mae'])

# now execute the q learning
y = 0.95
eps = 0.5
decay_factor = 0.999
r_avg_list = []
num_episodes = 10

for i in range(num_episodes):
    s = env.reset()
    eps *= decay_factor
    if i % 100 == 0:
        print("Episode {} of {}".format(i + 1, num_episodes))
    done = False
    r_sum = 0
    while not done:
        if np.random.random() < eps:
            a = np.random.randint(0, 2)
        else:
            a = np.argmax(model.predict(np.identity(5)[s:s + 1]))
        new_s, r, done, _ = env.step(action=a)
        target = r + y * np.max(model.predict(np.identity(5)[new_s:new_s + 1]))
        target_vec = model.predict(np.identity(5)[s:s + 1])[0]
        target_vec[a] = target
        model.fit(np.identity(5)[s:s + 1], target_vec.reshape(-1, 2), epochs=1, verbose=0)
        s = new_s
        r_sum += r
    r_avg_list.append(r_sum / 1000)

