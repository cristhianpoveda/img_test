#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
#dt-exec rosrun preproc test_node _image_transport:=compressed
#dt-exec roslaunch preproc tes_node.launch
#dt-exec rosrun preproc px_test_node
#dt-exec roslaunch path_planning coordinator_node.launch
#dt-exec roslaunch path_planning stop_finder_node.launch veh:="$VEHICLE_NAME"
dt-exec roslaunch param node.launch veh:="$VEHICLE_NAME"

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
