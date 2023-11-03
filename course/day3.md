# Welcome to Day 3 of Start Creating Robots 
If you have got this far, congrats! 

The hard part is over, now we get to start playing with robots. We will first start by teleoperation. 

# Start the simulation

Make sure Docker Desktop is running and then open VS Code. Look in the bottom left to make sure VS code says "Dev Container: Start Creating Robots Email" in the bottom left. If it doesn't, hit ctrl-shift-p to search and run the "Open Folder In Container Command". 

Now hit ctrl-shift-p and search for "Run Task". This time choose task named "Day 3".

You should be presented with two windows. One window is a simulation of a coffee shop world with a blue robot in the middle. The other window is a controller for driving the robot. 

Gazebo is like the world, and this robot is now like a remote controlled car. You can zoom in and out in Gazebo with your controls and use the teleoperation window for driving the robot.

## Take some time to play with the simulation. 

Notice the number in the bottom right of the screen, this number is the real-time percentage of your simulation. 

If its at or near 100% this means that the simulation is running at realtime, if its less than 100% 
then the simulation is running slower than real time. For example, 30% means every 3 real world seconds, only 1 second elapses in the simulated world. It should be running pretty fast right now, but tomorrow it will be different.

That's all I wanted to show you today, but if you want to dive a little deeper, there are two other things I'd like to highlight. 

# Launching Your Robot System

If you want to see how this system is running, check out the file: 

`src/start_creating_robots/launch/gazebo.launch.py`

This is a ROS 2 launch file, and it describes the nodes that are currently running this simulation. Over the next 3 days we will be running a different launch file, but they all build upon each other, so checking them out will give you a glimpse of what is needed to get a robot simulation running.
The Day 2 run task, simply builds the system and then runs this launch file. If you want to check out the script it runs, you can find it in /workspace/.vscode/scripts/day3.sh 

# Describing your robot

ROS and Gazebo use several different modelling languages to describe their worlds and robots. For our simulation they are in the folder:

`src/start_creating_robots/worlds_and_models/`

Our robot is described by the file:

`src/start_creating_robots/worlds_and_models/krytn/krytn.urdf.xacro`

If you're abit more experienced you might note that krytn.urdf.xacro has an option to disable sensors. See if you can find where this option is called in the gazebo.launch.py file. 



See you tomorrow! 

John
