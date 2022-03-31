# Manager Package

## 1. Installation

<!-- ```buildoutcfg
chmod +x ./install.sh && ./install.sh
```… -->
## 2. Description

RoboCup manager based on [Petri Net Plans](https://sites.google.com/a/dis.uniroma1.it/petri-net-plans/), running with ROS Melodic.

Exhaustive documentation available [there](https://docs.google.com/document/d/1k9010Ih97Cr6_wcB3hjiyZzdf_JtrkaXPFaJAhTO36g/edit)

> "Petri Net Plans (PNP) is a formalism for high level description of complex plans (i.e., set of actions interacting in a complex way). PNP is useful to program cognitive agents (such as robots, video game agents, multi-robot/multi-agent systems, etc.)."

Example of a Petri Net from PNP Documentation:
![alt text](https://raw.githubusercontent.com/iocchi/PetriNetPlans/master/PNPros/examples/rp_example/rp_pnp/plans/sensing.png "Logo Title Text 1")

**Currently available inside a Docker container.**

## 3. Usage

Two principle parts are necessary to make it work on a real world task:
- Petri Net modelisation and generation
- Development of high level actions and conditions using ROS.

The current version is encapsulated on a Docker container, you can build it using:
```
docker build --tag demo_pnp_noetic .
```

You can launch the Docker container using this command
```
docker run -it --rm --net=host \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-e DISPLAY=$DISPLAY \
demo_pnp_noetic
```


### 3.1 Petri Net Modelisation and Generation

A RoboCup task can quickly become complicated to design, especially when you have a lot of execution rules and exceptions to take into account.

It's possible to generate a custom Petri Net using high level rules.

**Example for The Carry My Luggage task**

Write plan file describing high level tasks required to finish the task in a certain orders. Multiple orders and possibilities can be added on a single file. File available [there](https://github.com/RoboBreizh-RoboCup-Home/manager_pepper/blob/devel/demo_pnp/plans/carry_my_luggage.plan).
We can then generate a first Petri Net using these high level tasks.

```
# Inside the Docker container
roscd demo_pnp/plans
pnpgen_linear carry_my_luggage.plan
```
You'll obtain a file called **carry_my_luggage.pnml** available [there](https://github.com/RoboBreizh-RoboCup-Home/manager_pepper/blob/devel/demo_pnp/plans/carry_my_luggage_without_exectution_rules.pnml).

For complex tasks, in addition with these high level tasks, we can add execution rules.

> "Execution Rules (ER) are a set of rules that are applied during the execution of a plan and which is not contemplated previously. Therefore, the use of ER simplifies the addition of special cases that otherwise would need to be encoded in the planning domain."

They are formed this way:
```
*if* <condition> *during* <action> *do* <recoveryplan>
```

For our **Carry My Luggage** task, we chose the rules available [there](https://github.com/RoboBreizh-RoboCup-Home/manager_pepper/blob/devel/demo_pnp/plans/carry_my_luggage.er).
We then generated a more complex plan which takes into account these rules:
```
# Inside the Docker container
roscd demo_pnp/plans
pnpgen_linear carry_my_luggage.plan carry_my_luggage.er
```
You'll obtain a complex Petri Net available [there](https://github.com/RoboBreizh-RoboCup-Home/manager_pepper/blob/devel/demo_pnp/plans/carry_my_luggage.pnml).

You can visualise these Petri Nets using **jarp** (Petri Net editor):
```
# Inside the Docker container
cd /home/robot/src/PetriNetPlans/Jarp
./jarp.sh
```

### 3.2 ROS Actions and conditions
**THIS PART IS STILL AN EARLY WIP**

**This is still a WIP, all the steps inside the petri net once launched will be done in a instant, lots of modifications needed.**

In order to test the different features offered by The ROS actions and conditions, we've created a simple Petri Net to test the development of the different actions.
We'll use the **plan_test_ros** Petri Plan and the actions developped in **DemoActions** C++ code.

To launch a plan, you'll need two terminals:
- First terminal used to launch Petri Net Plan manager and actions/conditions manager
```
# Launch PNP Docker
cd manager_pepper
sudo docker run -it --rm --net=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=$DISPLAY -v $(pwd):/home/robot/src/demo_pnp demo_pnp
cd /home/robot/ros/catkin_ws

# Build package
catkin_make

# Launch Petri Net module with Demo Actions
roslaunch demo_pnp launch_demo.launch
```
Everything should initialise correctly, "Waiting for a plan..." should be written in the end of the intialisation process.

- In a second terminal, you can publish on topic **/pnp/planToExec** to indicate which Petri Plan the manager must use. This petri plan must be stored inside **demo_pnp/plans** folder. We'll use the plan called "plan_test_ros" here.
```
# If you want to use a ROS Noetic installation on your computer
rostopic pub /pnp/planToExec std_msgs/String "data: 'plan_test_ros'" --once

# If you prefer to use a Docker container
docker run --rm -it --net host --name deckard_ros_noetic ros:noetic rostopic pub /pnp/planToExec std_msgs/String "data: 'plan_test_ros'" --once
```

You can stop the plan using:
```
# If you want to use a ROS Noetic installation on your computer
rostopic pub /pnp/planToExec std_msgs/String "data: 'stop'" --once

# If you prefer to use a Docker container
docker run --rm -it --net host --name deckard_ros_noetic ros:noetic rostopic pub /pnp/planToExec std_msgs/String "data: 'stop'" --once
```

## 4. Roadmap

- [X] Switch to ROS Noetic
- [ ] Add Actions support
- [ ] Add Conditions support 
- [ ] Add Actions interruption support
- [ ] Test the three features above in a basic scenario
- [ ] Test the manager in a complex task, e.g. Carry My Luggage