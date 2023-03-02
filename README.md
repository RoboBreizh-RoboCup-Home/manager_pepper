# Manager Package

## 1. Installation

```bash
git submodule update --init --recursive
```

## 2. Description

RoboCup manager based on [Petri Net Plans](https://sites.google.com/a/dis.uniroma1.it/petri-net-plans/), running with ROS Noetic.

Exhaustive documentation available [there](https://docs.google.com/document/d/1k9010Ih97Cr6_wcB3hjiyZzdf_JtrkaXPFaJAhTO36g/edit)

> "Petri Net Plans (PNP) is a formalism for high level description of complex plans (i.e., set of actions interacting in a complex way). PNP is useful to program cognitive agents (such as robots, video game agents, multi-robot/multi-agent systems, etc.)."

Example of a Petri Net from PNP Documentation:
![alt text](https://raw.githubusercontent.com/iocchi/PetriNetPlans/master/PNPros/examples/rp_example/rp_pnp/plans/sensing.png "Logo Title Text 1")

## 3. Usage

Two principle parts are necessary to make it work on a real world task:
- Petri Net modelisation and generation.
- Development of high level actions and conditions using ROS.

### 3.1 Petri Net Modelisation and Generation

A RoboCup task can quickly become complicated to design, especially when you have a lot of execution rules and exceptions to take into account.

You can visualise these Petri Net modelisations using the **Jarp** editor available on the Petri Net Plans repo:
```
# In your computer
git clone https://github.com/iocchi/PetriNetPlans.git
cd PetriNetPlans/Jarp
./jarp.sh
```

Petri Plans are stored in the **demo_pnp/plans** folder.

### 3.2 Functionnality test with GPRS scenario
```
# Launch Petri Net module with Demo Actions on the robot
roslaunch manager_pepper robobreizh_manager.launch
```

Everything should initialise correctly, "Waiting for a plan..." should be written in the end of the intialisation process.

In a second terminal, you can publish on topic **/pnp/planToExec** to indicate which Petri Plan the manager must use. Petri Plans must be stored inside **plans** folder. We'll use the plan called "GPSR" here.
```
# If you want to use a ROS Noetic installation on the robot or on your computer
rostopic pub /pnp/planToExec std_msgs/String "data: 'GPSR'"
```

## 4. Troubleshooting compiling errors
If you can't compile, you can try adding some paths to your bashrc (or the Gentoo Equivalent)
```
echo "export PNP_HOME=/home/nao/.local/PetriNetPlans" >> ~/.bash_profile
echo "export PNP_INCLUDE=${PNP_HOME}/PNP/include/" >> ~/.bash_profile
echo "export PNP_LIB=/home/nao/.local/bin/usr/local/lib/" >> ~/.bash_profile
```

Then, restart your SSH terminal to take the changes into account.


## 5. How to register actions and conditions

Actions and conditions must be registered on the main file available in [src/RoboBreizhManager.cpp](https://github.com/RoboBreizh-RoboCup-Home/manager_pepper/blob/devel/src/RoboBreizhManager.cpp).

## 6. Other commands

You can stop the plan before its end using:
```
**If you want to use a ROS Noetic installation on the robot or on your computer**
rostopic pub /pnp/planToExec std_msgs/String "data: 'stop'" --once
```
