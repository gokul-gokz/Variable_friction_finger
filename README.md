# Variable_friction_finger
## Steps to run the code
1. Download the repository into an workspace.
2. catkin_make the workspace. This will create dev and build files for the common_msgs_gl and rosserial files.
3. Then run the command 
> rosrun rosserial_client make_libraries path_to_library 
4. This will create a ros_lib foder on the given path.
5. Now open the arduino installation folder. Remove the existing ros_lib folder which will be inside the libraries folder.
6. Then move the newly created ros_lib folder to this location.
7. Copy the arduino code and burn it to the arduino board.
8. Check whether the arduino is connected to ACM0 port and the baud rate is set as 57600.
9. Run the command 
> rosrun rosserial_python serial_node.py
10. If the serial node is launched properly, you should see the msgs Setup service server on Friction_surface_Right [common_msgs_gl/SendBool] ,Setup service server on Friction_surface_Left [common_msgs_gl/SendBool] on the screen.
11. Now you can set the friction surface by running the command :
> rosservice call /Friction_surface_Right "data: true"
to set the friction surface high.
12. Similarly for the left finger , call the command:
>rosservice call /Friction_surface_Left "data: true"



