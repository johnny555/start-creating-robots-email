Welcome to Day 5 of Start Creating Robots 

It's time to build an autonomous robot!



Open VS Code, make sure docker is running and open the dev container. Look for a blue box in the bottom left that says "Dev Container: Start Creating Robots" to know you are good to go. If it doesn't say that hit Ctrl+Shift+P and run open folder in dev container.

Ok, lets run the day 5 code. Hit Ctrl+Shift+P and search for Run Task. Execute that and run Day 5. 

# You First Need A Map

Lets do a quick repeat of yesterday and teleoperate around. 

This should fill out the map alittle bit. You don't actually need a complete map, but its much nicer if the map is somewhat complete.

A Costmap Is Used To Help Your Robot Plan

Today's RViz is already setup with several maps. In the left pane you should see three. The Map, a Local Costmap and a Global Costmap.

Try clicking the check-box next to each map on and off separately to see what they look like. You should find that: 

- The Map is just the map. With thin black lines indicating the walls. 
- The Global Cost map looks just like the Map, but every wall has been expanded to be a bigger region. 
- The Local Cost map is just a really small square around the robot. 

Try driving the robot towards a wall and watch what happens to the Local cost map.

The big idea with the costmap is that when you are planning a path through the world, you want the robot to try to avoid hitting the walls. So the costmaps inflate the size of obstacles to help avoid that. Also, the global costmap allows a planner to work out the high level path, whereas the local planner allows the robot to make minor course corrections along the way, and deal with any unexpected obstacles. 

Alright, we are ready to start autonomously driving, first make sure to stop your robot so it doesn't get conflicting drive commands!

# Give Your Robot A Command

In the top of RViz you should see a button that says "2D Goal Pose". 

Click that button and then select somewhere on the map. You should see a green line formed between your robot and the target goal. The robot will now slowly drive to that location. Try asking it to go to different parts of the cafe. You should see it attempt to navigate around obstacles. 

Congratulations! You have successfully built an autonomous robot simulation.
You're Now Ready To Start Creating ROBOTS! 

It might seem like all you've done is click some buttons, but you actually have something very rare on your computer right now. 

A fully setup ROS simulation that you can change and modify to create any kind of robot. Feel free to dig into the launch files, the URDF and anything else. If you ever get stuck or break something, don't worry. You can always re-clone this repo and come back to this starting position. 

Good luck and please share what you build with me on LinkedIn (​https://www.linkedin.com/in/johnvial/​) and X/Twitter (​https://twitter.com/JohnVial​).


# Extra Credit: Look under the hood

Todays work relies purely on the ROS2 Nav2 software. 

Looking into the launch file at /workspace/src/start_creating_robots/launch/navigation.launch.py you should see that there isn't much to it. We just include everything from the mapping system and start the navigation system. This hides complexity, as the ROS2 Navigation stack is very sophisticated. 

You can get a sense of this by checking out the config file at /workspace/src/start_creating_robots/config/navigation.yaml

# Want to keep creating robots? Let me know what you need to know next!

Congratulations for getting this far. Most people give up waay before they get a working ROS system. 

I'd love to help you more, I'm putting together a book and online course, and if that's something your interested in I'd love to have you in the course. Please fill out this survey (​https://forms.gle/k7LZZSrbK55YwbwR8​) to let me know what you would like to learn about next!

I'm also running a 4 week robotics simulation challenge. If that's sounds fun to you, you can sign up here: 

https://start-creating-robots.ck.page/products/4-week-robotics-challenge



Cheers!

John
