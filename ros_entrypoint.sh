#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# setup workspace if it exists
if [ -n "$WORKSPACE_NAME" ]; then 
    if [ ! -e "/root/$WORKSPACE_NAME/devel/setup.sh" ]; then
        previousDirectory=$(pwd)
        cd /root/$WORKSPACE_NAME
        catkin_make
        cd $previousDirectory
    fi
    source "/root/$WORKSPACE_NAME/devel/setup.sh"
fi

source "/root/workspace/devel/setup.bash"

exec "$@"

