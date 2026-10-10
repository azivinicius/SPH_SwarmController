# Source this instead of "install/setup.bash" directly.
#
# Workaround: colcon isn't classifying our packages as "ament_python" in
# this container. Two things that build type would normally do
# automatically don't happen: (1) adding the package's install folder to
# AMENT_PREFIX_PATH, and (2) exposing its console_scripts executables
# under lib/<package>/, which is where "ros2 run" and "ros2 launch" both
# look. This script does both by hand. Add one block per new package.

source /root/dev_ws/install/setup.bash

# --- drone_controller ---
export AMENT_PREFIX_PATH="/root/dev_ws/install/drone_controller:$AMENT_PREFIX_PATH"
mkdir -p /root/dev_ws/install/drone_controller/lib/drone_controller
ln -sf /root/dev_ws/install/drone_controller/bin/plain_sph \
       /root/dev_ws/install/drone_controller/lib/drone_controller/plain_sph
ln -sf /root/dev_ws/install/drone_controller/bin/swarm_sph \
       /root/dev_ws/install/drone_controller/lib/drone_controller/swarm_sph
ln -sf /root/dev_ws/install/drone_controller/bin/particle_sim \
       /root/dev_ws/install/drone_controller/lib/drone_controller/particle_sim