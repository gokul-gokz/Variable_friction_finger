# Variable_friction_finger
## Steps to run the code
1. Download the repository into an workspace.
2. catkin_make the workspace. This will create dev and build files for the common_msgs_gl and rosserial files.
3. Copy the arduino code and burn it to the arduino board.
4. Check whether the arduino is connected to ACM0 port and the baud rate is set as 57600.
5. Run the command: rosrun rosserial_python serial_node.py . If the serial node is launched properly, you should see the msgs Setup service server on Friction_surface_Right [common_msgs_gl/SendBool] ,Setup service server on Friction_surface_Left [common_msgs_gl/SendBool] on the screen.
6. Now you can set the friction surface by running the command : rosservice call /Friction_surface_Right "data: true" to set the friction surface high. Similarly for the left finger , call the command: rosservice call /Friction_surface_Left "data: true"



