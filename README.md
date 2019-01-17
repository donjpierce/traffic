# traffic
An abstract and minimilist traffic simulation, using OpenStreetMaps and OSMnx, can simulate traffic on any road network from OSM. 
By using a Keras three-layer, linear learning architecture, with TensorFlow backend, an optimization algorithm can be implemented for one car in the system.
This car will learn its pre-configured route to shortest-time by trying an alternate route which avoids apparent obstacles (such as traffic lights or heavy traffic).
The learning algorithm uses Q-learning with epsilon-greedy techniques, and a dynamic reward table based on a car's time-to-complete.


Usage:

After customizing desired parameters, run the `artist` scratch file with `python artist.py` to render .mp4 movies of a traffic simulation.
Or after selecting a learning agent from the available cars, run `python learn.py` to optimize that car's route to shortest-time.




![alt text](https://raw.githubusercontent.com/donjpierce/traffic/master/examples/piedmont33cars.gif)
![alt text](https://raw.githubusercontent.com/donjpierce/traffic/master/examples/lowerManhattan.gif)

**Simple Example of Optimization**

![alt text](https://raw.githubusercontent.com/donjpierce/traffic/master/examples/car_learn.png)
The above figure is a simple example of how a learning agent reroutes around a single traffic light. 

In the original route (left) the learning agent is in state 1. State 1 implies three criteria:
(1) there are obstacle(s) in the route, (2) the detour around the most significant obstacle is short, (3) the detour contains no obstacle(s). (See the Projects tab of this repository to learn about all 10 possible
states in the Markov decision chain for this project.)

The alternate route (right) is traveled in lieu of the original if the learning agent takes Action 1 (Action 0 is to do nothing). 

The reward-scheme for this learning algorithm is based on whether the time to travel from A to B is the shortest of the two
action possibilities. Since the alternate route above does not experience a traffic light, taking Action 1 (i.e. rerouting) will be preferred the more the learning agent performs the simulation.

![alt text](https://raw.githubusercontent.com/donjpierce/traffic/master/examples/avg_rewards_decay0.99.png)

Sure enough, the average reward per game improves while training for 90-games (above). The reason the average reward per game 
asymptotes as the number of games increases is because the learning agent eventually only chooses Action 1 (as it should for highest reward).

**Special Case: Car 17 Route, Alternate Route, and Learning Process**

![alt text](https://raw.githubusercontent.com/donjpierce/traffic/master/examples/avg_rewards.png)

![alt text](https://raw.githubusercontent.com/donjpierce/traffic/master/examples/car17_learn.png)

**Rewards**
As can be seen in the above plot, a purely time-based reward system for this learning algorithm does not always converge on a route with shortest-time. In the case of car no. 17,
this is because the arrival times of the original route and the alternate route were both very similar. A new reward system is needed, one based on the positivity of the time difference, for convergence.

**A note on the requirements for this code base**

Installing 

	tensorflow 1.9.0 py36_0 (latest 12/29/2018) 

conflicts with any python 3.7 (py37) environment because tf uses `python >=3.6,<3.7.0a0`. To handle this, I've created another environment with all the python 3.6 (py36) versions of the required packages for this code base. To install this py36 environement, see `requirements_py36.txt` in the requirements folder. 

Once TensorFlow is upgraded to py37, the py36 requirements will be depricated.
