# Perception Package

## 1. On the robot : start naoqi with ROS

```buildoutcfg
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=IP_ROBOT network_interface:=eth0

```
## 2. On the robot : source ros pkg

```buildoutcfg
cd ~/WORK_SPACE
source devel/setup.bash

```
## 3. On the robot : CV

```buildoutcfg
cd ~/perception-pepper/scripts
python roboBreizh_CV_Detector.py

```
## 3. On the robot : Dialog

```buildoutcfg
cd ~/perception-pepper/scripts
python roboBreizh_DescribePerson.py

```
## 4. On the PC : viewers
```buildoutcfg
export ROS_MASTER_URI=http://pepper2.local:11311/
cd ~/perception-pepper/scripts
python roboBreizh_Viewer_CV.py
python roboBreizh_ViewerKmeans.py

```
