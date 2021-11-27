# **ROS-Turtlebot-with-optical-flow Manual** 

​																					**2018102094  소프트웨어융합학과 김대호**    

​																										     Ubuntu 16.04 , ROS Kinetic 

1. ```$ roscore ```  	

   Run 'roscore'  in terminal.

2. ```$ rosrun uvc_camera uvc_camera_node```

   Run your web camera.

   If you use VMware , connect your camera to VMware.

3. ```$ roslaunch turtlebot3_gazebo turtlebot3_world.launch``` 

   If  '[gazebo_gui-2]' error occur, ``` $export libgl_always_software=1``` 

   And try again. Gazebo and turtlebot3 will appear. 

4. ``` $ source ~/catkin_ws2/devel/setup.bash```

   This workspace name is '**catkin_ws2**'. Not 'catkin_ws'. 

5. ``` $ rosrun middle_pkg center_node ```

   The window for Mission #1 and Mission #2 will appear. 

   First, check the results while moving the object in front of the camera.

   Second, check the results while moving the camera itself.

   




※ If the direction is reversed adjust the value in **'all.cpp'** .  You can adjust **'noise threshold'** value , **'background threshold'** value and **'move'** value. 

