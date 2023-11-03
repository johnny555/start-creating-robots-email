# Welcome to Day 4 of Start Creating Robots 


Today we are going to explore sensors and mapping. 









Just like yesterday you need to turn on Docker and make sure you have started VS Code within the Dev Container. 

Now type Ctrl+Shift+P search an run "Run Task", and now choose "Day 4".

# RViz - Your Robots Perspective

Today we are taking a look through the robots eyes.

In this new simulation, the robot has been equipped with a forward facing laser scanner and very small depth camera. You'll also notice a new window has opened on your computer titled RViz2. This is the ROS visualization software, and it lets you see what the robot is seeing.

Zoom around in the RViz interface, you should notice red dots representing lidar returns and some small pixels representing colourized depth measurements. 
Teleoperate the robot around looking only at RViz and the teleoperation window. Occasionally peak at Gazebo if you get stuck. This will simulate how driving a robot in the real world will be. Most of the time you'll either be looking through its sensors, or actually looking at it to figure out if what you are seeing on the sensors makes sense.

# Visualise the map

While you have been driving around, the robot has been secretly making a map of its environment using the lidar. You can add this map to the visual by clicking on the add button in the bottom of the left-hand panel of RVIZ.

A popup window should appear, choose the "By Topic" tab to select a visualisation by topic. Next find the topic /map and select the grey icon below it that says "Map". 
Congratulations, you have created visualised the map. 

# Changing to the map coordinate frame

Drive the robot around and what what happens to the map. You should see that the robot is always in the centre of the map, and the map will update around the robot. This is pretty weird, and not at all how we think about maps. 

This is happening because RViz is setup for showing the work with respect to the robots frame of reference. To get the map to behave abit more sensibly, go to  the top of the left pane and look for "Global Options". Below it you see a field "Fixed Frame", change this field to "/map".

Now drive the robot around, you should find that the robot moves in the map, and as the map updates the robots location snaps to where it should be. 
Also compare what you see in Gazebo to the map of the world. Is it accurate? 

Ok that's it for Day 4.

# Extra Credit - Under The Hood 

If you want to understand whats going on abit deeper, look into the new launch file: 
/workspace/src/start_creating_robots/launch/mapping.launch.py 
This launch file includes the gazebo.launch.py from yesterday, but has some substantial changes. Most notably we are bridging alot more data from gazebo with the extended_bridge node. This allows us to get all of the sensor data as well as the transformations published by the gazebo motor controller. These transforms help ROS to know where all the parts of the robot are. 

We are also launching the slam_toobox to provide the mapping and a node to start RViz. 

# Go Even Deeper - Looking at Parameters 

The SLAM Toolbox node is very complicated, and takes lots of parameters. 

Rather than describe them all in the python file, they are loaded in from the file:
/workspace/src/start_creating_robots/config/mapping.yaml
You don't need to know what they do, but it's good to know where your parameters live if you ever want to change them. 
Creating Robots First Step - Modify Your Sensors
You might be thinking, "Why is my camera so small?" or "I wish my lidar had more dots!". 
Thanks to the wonders of simulation, you can change the sensor parameters. If you want to change the field of view of the depth camera, try changing the <horizontal_fov> tag in:
/worspace/src/start_creating_robots/worlds_and_models/krytn/realsense_d435.urdf.xacro
Alternatively, look at lidar_2d_v1.urdf.xacro in the same folder and change the parameters within the <scan> tag to change your lidar scan.
Hit ctrl-c in the terminal that opened when you ran todays task, and then re-run the Day 4 task to regenerate your robot model with the upgraded sensors. 
Just watch out though, more sensor data means your simulation will run slower. Keep an eye on the real time speed percentage we looked at in Day 3 to know if its running slow. I prefer to keep my simulations above 30% if possible. 

See you tomorrow! 


John


