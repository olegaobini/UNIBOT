# UNIBOT

This is a student developed self-driving self-balancing unicycle, controlled with lidar navigation.


# TODO
- print out new side plate with elongated screw slots. The main motor belt needs more tension.
- print out a new top plate with hole cutouts for the raspberry pi 5
- find a remote controller that I could use to control everything.
- add screws to the reaction wheel's to increase mass.

# Control File Directory
uni_control/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── control.launch.py         # Optional: launch file for control node(s)
├── config/
│   └── control_params.yaml       # PID gains, filter params, motor limits
├── include/uni_control/
│   └── balance_controller.hpp    # Class for computing motor commands
│   └── imu_processor.hpp         # Complementary filter, calibration
│   └── motor_driver.hpp          # Interface to motors (PWM, UART, etc)
├── src/
│   └── balance_controller.cpp    # Implements balancing logic
│   └── imu_processor.cpp         # Handles IMU reading/filtering
│   └── motor_driver.cpp          # Sends commands to motors
│   └── control_node.cpp          # ROS2 node that ties everything together
└── README.md