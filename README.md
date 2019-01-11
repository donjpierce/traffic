# traffic
An abstract and minimilist traffic simulation, using OpenStreetMaps and OSMnx, can simulate traffic on any road network from OSM. 
By using a Keras three-layer, linear learning architecture, with TensorFlow backend, an optimization algorithm can be implemented for one car in the system.
This car will learn its pre-configured route to shortest-time by trying an alternate route which avoids apparent obstacles (such as traffic lights or heavy traffic).
The learning algorithm uses Q-learning with epsilon-greedy techniques, and a dynamic reward table based on a route's time-to-complete.


Usage:

After customizing desired parameters, run `python animate.py` to render .mp4 movies of a traffic simulation.
Or after selecting a learning agent from the available cars, run `python learn.py` to optimize that car's route to shortest-time.




![alt text](https://raw.githubusercontent.com/donjpierce/traffic/master/examples/piedmont33cars.gif)
![alt text](https://raw.githubusercontent.com/donjpierce/traffic/master/examples/lowerManhattan.gif)
![alt text](https://raw.githubusercontent.com/donjpierce/traffic/master/examples/avg_rewards.png)

**Rewards**
As can be seen in the above plot, the reward system for this learning algorithm does not always converge on a route with shortest-time. In the case of car no. 17,
this is because both the original and alternate routes contained traffic lights (i.e. there is little difference between the original route and a calculated detour for car no. 17).

**A note on the requirements for this code base**

Installing 

	tensorflow 1.9.0 py36_0 (latest 12/29/2018) 

conflicts with any python 3.7 (py37) environment because tf uses `python >=3.6,<3.7.0a0`. To handle this, I've created another environment with all the python 3.6 (py36) versions of the required packages for this code base. To install this py36 environement, see `requirements_py36.txt` in the requirements folder. 

Once TensorFlow is upgraded to py37, the py36 requirements will be depricated.
