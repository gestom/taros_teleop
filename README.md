## Cameleon teleop package

This package allows to take manual control of the cameleon robot at any time. The package consist of two nodes. First node, *named joy_node*, reads data from joystick a sends them to the *cameleon_teleop* node, which processes the incoming data a sends them to *Cameleon ROS driver*. When the robot is not controlled manually, the *cameleon_teleop* simply receives commands over the *cmd* and *flipperVelocity* topics and passes them over to the *cameleon/cmd_vel*. When the operator presses *manualOverrideButton* (4 by default), the *cmd* and *flipperVelocity* messages are discarted and the operator can drive the robot manually. 

To drive the robot manually, the operator has to keep the *manualOverrideButton* pressed.
Then, the operator can set the robot forward acceleration (i.e. moving the joystick forwards and backwards causes the robot to increase or decrease speed), turning speed (i.e. moving the joystick sideways causes the robot to turn) and flipper speed (additional axis of the joystick can cause the flippers to move (counter-)clockwise.

To use this package:
1. Install joystick package: _sudo apt-get install ros-kinetic-joy_
1. Plug in your joystick and check which device it appears as, it should be something like _ls /dev/input/jsX_
1. Use the _jscal_ package to determine which axes you want to use to drive the robot.
1. Edit the _cameleon_teleop.launch_ file to set the proper device and axes.
1. Run the _cameleon_teleop.launch_.
