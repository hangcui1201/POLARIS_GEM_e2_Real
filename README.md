### Starter Code of Polaris GEM e2 for [ECE484](https://publish.illinois.edu/safe-autonomy/) and [CS588](http://luthuli.cs.uiuc.edu/~daf//courses/MAAV-22/588-2022-home.html)

#### Center for Autonomy at University of Illinois Urbana-Champaign

#### Author: Hang Cui (hangcui3@illinois.edu)

#### System: Ubuntu 20.04 + ROS Noetic

#### Polaris GEM e2 Vehicle

<a href="url"><img src="./images/Polaris_GEM_e2.png" width="600"></a>  

<a href="url"><img src="./images/GEM_Rviz1.png" width="600"></a>  

<a href="url"><img src="./images/GEM_Rviz2.png" width="600"></a>  

#### Demo

$ cd ~/demo_ws/  
$ catkin_make  

$ source devel/setup.bash  
$ roslaunch basic_launch sensor_init.launch  

$ source devel/setup.bash  
$ roslaunch basic_launch visualization.launch  

$ source devel/setup.bash  
$ roslaunch basic_launch dbw_joystick.launch  

#### GNSS-based waypoint follower with Stanley controller and RTK enabled

Click the video below.  

[![](http://img.youtube.com/vi/DItwU_8GVHI/0.jpg)](https://youtu.be/DItwU_8GVHI "GNSS-based waypoint follower with Stanley controller and RTK enabled")  

$ source devel/setup.bash  
$ roslaunch basic_launch gem_pacmod_control.launch  

$ source devel/setup.bash  
$ rosrun gem_gnss_control gem_gnss_tracker_stanley_rtk.py  

#### GNSS-based waypoints follower with Pure Pursuit controller

Click the video below.  

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/8l52buLR1zU/0.jpg)](https://www.youtube.com/watch?v=8l52buLR1zU "GNSS based waypoints following using pure pursuit controller")  

$ source devel/setup.bash  
$ rosrun gem_gnss_control gem_gnss_tracker_pp.py  

#### Vision based (monovision camera) lane following using Stanley controller

Click the video below (still under development).  

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Ns7aXRgEP2g/0.jpg)](https://www.youtube.com/watch?v=Ns7aXRgEP2g "Vision based (monovision camera) lane following using Stanley controller")  

