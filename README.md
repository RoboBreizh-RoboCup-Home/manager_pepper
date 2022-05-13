# Manager Package - Test version without a robot
Only meant to be used to test manager functionnalities


The current version is encapsulated on a Docker container, you can build it using:
```
docker build --tag robobreizh_manager_docker_test .
```

You can launch the Docker container using this command
```
docker run -it --rm --net=host robobreizh_manager_docker_test
source /root/workspace/devel/setup.bash
```

You can start the MongoDB Database handler using (will be added to the manager roslaunch), roscore must be launched elsewhere before ATM
```
roslaunch mongodb_store mongodb_store.launch db_path:=/opt/ros/robobreizh_mongodb_store
```

