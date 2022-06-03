FROM ros:noetic

SHELL ["/bin/bash", "-c"] 

COPY ./ros_entrypoint.sh /

RUN apt-get update && \ 
    apt-get install -y g++ cmake libxml2 libxml2-dev flex git && \
    apt-get clean 

RUN mkdir src && \
    cd src && \
    git clone https://github.com/iocchi/PetriNetPlans.git

RUN cd src/PetriNetPlans/PNP && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    sudo make install

RUN apt-get -y install libboost-all-dev libxml++2.6-dev libpcre3-dev && \
    cd src/PetriNetPlans/PNPgen && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    sudo make install && \
    sudo ldconfig

RUN apt-get update && \
    apt-get -y install ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-msgs libmove-base-msgs-dev python-is-python3 apt-utils sqlite

COPY ./workspace /root/workspace

RUN cd /root/workspace/src && \
    ln -s /src/PetriNetPlans/PNPros/ROS_bridge/pnp_ros . && \
    ln -s /src/PetriNetPlans/PNPros/ROS_bridge/pnp_msgs . && \
    git clone -b kinetic-devel https://github.com/ros-planning/warehouse_ros.git && \
    git clone -b master https://github.com/ros-planning/warehouse_ros_sqlite.git && \
    source /opt/ros/noetic/setup.bash && \
    cd .. && \
    catkin_make

    
