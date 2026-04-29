#/bin/bash
git submodule update --init --recursive --remote 

rosdep install --from-paths src --ignore-src --default-yes

source /opt/prebuilt/slic3r_coverage_planner/setup.bash && catkin_make -DCATKIN_BLACKLIST_PACKAGES="mower_comms_v2;mower_simulation"
