## Starter Code of Polaris GEM e2 for [ECE484](https://publish.illinois.edu/safe-autonomy/) and [CS588](http://luthuli.cs.uiuc.edu/~daf//courses/MAAV-22/588-2022-home.html) at University of Illinois at Urbana-Champaign

### Auther: Hang Cui (hangcui3@illinois.edu)

$ cd ~/demo_ws/
$ catkin_make

$ source devel/setup.bash
$ roslaunch basic_launch gem_sensor_init.launch

$ source devel/setup.bash
$ roslaunch basic_launch gem_dbw_joystick.launch

#### GNSS-based waypoint follower with Stanley controller and RTK enabled

$ source devel/setup.bash
$ roslaunch basic_launch gem_pacmod_control.launch

$ source devel/setup.bash
$ rosrun gem_gnss_control gem_gnss_tracker_stanley_rtk.py

#### GNSS-based waypoints follower with Pure Pursuit controller

$ source devel/setup.bash
$ rosrun gem_gnss_control gem_gnss_tracker_pp.py






