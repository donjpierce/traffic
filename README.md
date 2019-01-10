# traffic
minimilist traffic simulations using OpenStreetMaps and OSMnx


Usage:

After customizing desired parameters, run `python animate.py`




![alt text](https://raw.githubusercontent.com/donjpierce/traffic/master/examples/piedmont33cars.gif)
![alt text](https://raw.githubusercontent.com/donjpierce/traffic/master/examples/lowerManhattan.gif)

**A note on the requirements for this code base**

Installing 

	tensorflow 1.9.0 py36_0 (latest 12/29/2018) 

conflicts with any python 3.7 (py37) environment because tf uses `python >=3.6,<3.7.0a0`. To handle this, I've created another virtual environment with all the python 3.6 (py36) versions of the required package for this code base. To install this py36 environement, see `requirements_py36.txt` in the requirements folder. 

Once TensorFlow is upgraded to py37, this py36 environment will be depricated.
