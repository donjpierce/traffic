import matplotlib
from environment import Env
from keras import Sequential, layers
import matplotlib.pyplot as plt
import numpy as np
from osm_request import OGraph


def main(
        location="Upper West Side, Manhattan, New York City, New York, USA",
        cars=50,
        dt=1/1000,
        agent=17
):
    """

    :param location: str: a geo-codable place, like "Harlem, NY" or "Kigali, Rwanda"
    :param dt: float: time step for each frame
    :param cars: int: the total number of cars to simulate
    :param agent: int: the number of the car that will be the learning agent (must be less than cars)
    :return:
    """

    graph = OGraph(location, save=True)

    # initialize the environment for the learning agent
    env = Env(n=cars, graph=graph, agent=agent, dt=dt, animate=False)

    # initialize the Keras training model
    model = Sequential()
    model.add(layers.InputLayer(batch_input_shape=(1, 10)))
    model.add(layers.Dense(10, activation='sigmoid'))
    model.add(layers.Dense(2, activation='linear'))
    model.compile(loss='mse', optimizer='adam', metrics=['mae'])

    # now execute Q learning
    y = 0.95
    eps = 0.5
    decay_factor = 0.80
    num_episodes = 40

    r_avg_list = []
    r_sum_list = []

    file = open('diag.txt', 'w')

    for i in range(num_episodes):
        print("Episode {} of {}".format(i + 1, num_episodes))
        eps *= decay_factor
        r_sum = 0
        done = False
        diag_action = 0
        diag_reward = 0
        state = env.reset((i, num_episodes))
        while not done:
            env.reset((i, num_episodes))
            rand = np.random.random()
            if rand < eps:
                action = np.random.randint(0, 2)
            else:
                action = np.argmax(model.predict(np.identity(10)[state:state + 1]))
            new_s, r, done, _ = env.step(action=action, num=(i, num_episodes))
            target = r + y * np.max(model.predict(np.identity(10)[new_s:new_s + 1]))
            target_vec = model.predict(np.identity(10)[state:state + 1])[0]
            target_vec[action] = target
            model.fit(np.identity(10)[state:state + 1], target_vec.reshape(-1, 2), epochs=1, verbose=0)
            state = new_s
            r_sum += r
            print('Action: {}, Reward: {}'.format(action, r))
            file.write('Action: {}, Reward: {}'.format(action, round(r, 2)))
            diag_action += action
            diag_reward += r
        r_avg_list.append(r_sum)
        r_sum_list.append(sum(r_avg_list) / (i + 1))
        file.write('Episode: {}, Total Rewards: {} \n'.format(i, round(r_sum, 2)))

    file.close()

    plt.plot(np.arange(num_episodes), r_sum_list)
    plt.xlabel('Game number')
    plt.ylabel('Average reward per game')
    plt.suptitle('Average reward per game for car no. {}'.format(agent))
    plt.savefig('avg_rewards.png')

    return

