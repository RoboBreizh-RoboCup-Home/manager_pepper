#!/bin/bash
rostopic pub /pnp/planToExec std_msgs/String "data: '$1'" --once
