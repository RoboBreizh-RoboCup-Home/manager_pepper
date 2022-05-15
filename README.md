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
```
