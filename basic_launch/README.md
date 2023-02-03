## ROS Basic Launch files

Launch files for basic features provided by AutonoumouStuff for autonomy development vehicle platforms:

    - dbw_joystick.launch: Direct joystick control of drive-by-wire features such as steering, throttle, and braking

    - ssc_joystick.launch: Joystick control of the AutonomouStuff Speed and Steering Control Modules (requires an SSC license)  Instead of direct control of steering wheel angle, throttle pedal, and brake pedal, the joystick instead provides desired vehicle speed and desired steering curvature.

    - visualization.launch: Launches RVIZ and all installed sensors with visualization topics

These launch files assume that the platform_launch package has been installed, and that it has support for this specific vehicle.

They also assume that the following environment variables exist:

    - platform_name: The directory in platform_launch that supports this vehicle

