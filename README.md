# Manager Package

## 1. Installation

<!-- ```buildoutcfg
chmod +x ./install.sh && ./install.sh
```… -->

## TEMPORARY - COMMANDS TO REDACT NEXT VERSION OF README
```
new roslaunch : roslaunch manager_pepper robobreizh_manager.launch
launch plan: rostopic pub /pnp/planToExec std_msgs/String "data: 'GPSR'"
```
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

**You can install the manager in the robot by doing:**


```
# Other dependencies from Pepper OS using install_dependencies must be installed BEFOREHAND

# On your local computer:
git clone https://github.com/RoboBreizh-RoboCup-Home/manager_pepper.git
cd manager_pepper/workspace/src
scp -r robobreizh_demo_components/ demo_pnp/ nao@PEPPER_IP:~/robobreizh_ws/src/ 

# Connnect to the robot
ssh nao@PEPPER_IP
./install_dependencies.sh
```
**If you want to test the Manager locally, you can use the Docker container**

You can build it using:
```
docker build --tag demo_pnp_noetic .
```

You can launch the Docker container using this command
```
docker run -it --rm --net=host \
demo_pnp_noetic
```


### 3.1 Petri Net Modelisation and Generation

A RoboCup task can quickly become complicated to design, especially when you have a lot of execution rules and exceptions to take into account.

It's possible to generate a custom Petri Net using high level rules.

**Example for The Carry My Luggage task**

Write plan file describing high level tasks required to finish the task in a certain orders. Multiple orders and possibilities can be added on a single file. File available [there](https://github.com/RoboBreizh-RoboCup-Home/manager_pepper/blob/devel/demo_pnp/plans/carry_my_luggage.plan).
We can then generate a first Petri Net using these high level tasks.

```
# Inside the Docker container only, we don't need it on the robot
source /root/workspace/devel/setup.bash
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
# Inside the Docker container only, we don't need it on the robot
source /root/workspace/devel/setup.bash
roscd demo_pnp/plans
pnpgen_linear carry_my_luggage.plan carry_my_luggage.er
```
You'll obtain a complex Petri Net available [there](https://github.com/RoboBreizh-RoboCup-Home/manager_pepper/blob/devel/demo_pnp/plans/carry_my_luggage.pnml).

You can visualise these Petri Nets using **jarp** (Petri Net editor):
```
# Inside the Docker container only, we don't need it on the robot
/src/PetriNetPlans/Jarp
./jarp.sh
```
**If it doesn't work, you can directly clone the Petri Net Plans repo on your computer and use the same command locally**

### 3.2 ROS Actions and conditions

In order to test the different features offered by The ROS actions and conditions, we've created a simple Petri Net to test the development of the different actions.
We'll use the **plan_test_ros** Petri Plan and the actions developped in **DemoActions** C++ code.

The Petri Net used here is the following: 
![alt text](https://github.com/RoboBreizh-RoboCup-Home/manager_pepper/blob/devel/readme_ressources/demo_pnp_robobreizh.png "Demo PNP")

To launch a plan, you'll need two terminals:
- First terminal used to launch Petri Net Plan manager and actions/conditions manager
```
# If you use Docker
cd manager_pepper
docker run -it --rm --net=host  demo_pnp_noetic
source /root/workspace/devel/setup.bash

# If you use it directly installed on the robot
ssh nao@PEPPER_IP
source robobreizh_ws/devel/setup.bash

# Launch Petri Net module with Demo Actions
roslaunch demo_pnp launch_demo.launch
```
Everything should initialise correctly, "Waiting for a plan..." should be written in the end of the intialisation process.

- In a second terminal, you can publish on topic **/pnp/planToExec** to indicate which Petri Plan the manager must use. This petri plan must be stored inside **demo_pnp/plans** folder. We'll use the plan called "plan_test_ros" here.
```
# If you want to use a ROS Noetic installation on the robot or on your computer
rostopic pub /pnp/planToExec std_msgs/String "data: 'plan_test_ros'" --once

# If you prefer to use a Docker container
docker run --rm -it --net host --name deckard_ros_noetic ros:noetic rostopic pub /pnp/planToExec std_msgs/String "data: 'plan_test_ros'" --once
```

**3.2.1 Rostopic subscribe demo**

Then, the plan should indicate in the last line "Waiting for string input from /computer_vision/color". This means the demo plan is working fine: the plan is on the state checkColor.exec, has subscribed to the "/computer_vision/color" topic and is waiting for data.

You can send a string on this topic using:
```
# If you want to use a ROS Noetic installation on the robot or on your computer
rostopic pub /computer_vision/color std_msgs/String "data: 'grey'" --once

# If you prefer to use a Docker container
docker run --rm -it --net host --name deckard_ros_noetic ros:noetic rostopic pub /computer_vision/color std_msgs/String "data: 'grey'" --once
```

**3.2.2 Rosservice**

**This part (3.2.2) is currently not available**


You can also observe a demonstration of rosservice use inside the Petri Net Plan, in the demoRosService Action.
The **add_two_servers** node is instantiated in the same launch file. We can use the same principle for our vision demo scenario.
It uses the **add_two_servers** type and server included inside the test_docker package present in the same workspace.
You should observe the following logs:
```
### Executing demo_ros_service ... 
Returning [2 + 3 = 5]
2 + 3 = 5
### Finished demo_ros_service
```

The input values 2 and 3 are currently hardcoded, the next step is to add them as parameters in the Petri Net. This would allow us to reuse the same action server in multiple tasks.


## 5. Manager test onboard robot with NaoQi support
Scenario test with Pepper robot:
![alt text](https://github.com/RoboBreizh-RoboCup-Home/manager_pepper/blob/devel/readme_ressources/demo_pnp_pepper_robobreizh.png "Demo PNP")

```
# Can only be used on the robot
ssh nao@PEPPER_IP
source robobreizh_ws/devel/setup.bash

# Launch Petri Net module and other services
roslaunch demo_pnp launch_demo.launch
```

Then, you need to indicate which scenario you want to use:
```
rostopic pub /pnp/planToExec std_msgs/String "data: 'plan_test_ros_robobreizh'" --once
```

The initialisation part should go well (it's currently empty), then it'll ask you for a go signal:
```
rostopic pub /robobreizh/manager/go std_msgs/String "data: 'go'" --once
```

The robot should then speak. The next step is wait until there's someone close to the robot. Perception module isn't available yet. You can send Person message information to the "/robobreizh/perception/face_info" topic.
If it's not implemented yet, you can mock it by typing:
```
rostopic pub /robobreizh/perception/face_info robobreizh_demo_components/Person "{name: '', clothes_color: '', age: '', gender: '', skin_color: '', distance: 0.0}" --once
```
It should then indicate that someone has been seen on the manager terminal.
As shown on the graph, it'll loop and type on the terminal console "Found someone" each time it receives info from this type.

## 6. Vision/Speech proposed Petri Plan

Depending on what you want for the demonstration, we can for example decompose it this way:
![alt text](https://github.com/RoboBreizh-RoboCup-Home/manager_pepper/blob/devel/readme_ressources/proposition_cut_vision_demo_task.png "Demo PNP")

## 7. Other commands

You can stop the plan before its end using:
```
**If you want to use a ROS Noetic installation on the robot or on your computer**
rostopic pub /pnp/planToExec std_msgs/String "data: 'stop'" --once

# If you prefer to use a Docker container
docker run --rm -it --net host --name deckard_ros_noetic ros:noetic rostopic pub /pnp/planToExec std_msgs/String "data: 'stop'" --once
```

## 8. Roadmap

- [X] Switch to ROS Noetic
- [X] Add Actions publish/subscribe support
- [X] Add Actions ROS service support
- [X] Test the two features above in a basic scenario
- [X] Implement and make sure the manager works locally on the robot
- [ ] Add Actions input parameters support
- [ ] Test Action input parameter inside test scenario
- [ ] Add Conditions support 
- [ ] Add Actions interruption support
- [ ] Add the two features above in the basic scenario
- [ ] Test the manager in a complex task, e.g. Carry My Luggage