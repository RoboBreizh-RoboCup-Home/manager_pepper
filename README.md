# Manager Package - Test version without a robot
Only meant to be used to test manager functionnalities


The current version is encapsulated on a Docker container, you can build it using:
```
docker build --tag robobreizh_manager_docker_test .
```

You can launch the Docker container and test the manager using these commands
```
docker run -it --rm --net=host robobreizh_manager_docker_test
source /root/workspace/devel/setup.bash
roslaunch manager_pepper robobreizh_manager.launch
rostopic pub /pnp/planToExec std_msgs/String "data: 'GPSR'"
```

Demonstration has been made on the Initialisation plan action.

## Missing elements to make it work on the robot yet

**ROS packages**
```
# In the catkin workspace
git clone -b noetic-devel https://github.com/v4r-tuwien/mongodb_store.git
```
**Libraries**
- mongodb
- mongodb-dev

**Python dependencies**
- [pymongo](https://pypi.org/project/pymongo/)
- [future](https://pypi.org/project/future/)

I may forget some, you can check the Dockerfile if you have any problem.
