# Welcome to Day 2 of Start Creating Robots 

Today we are going to download and setup our environment. 

All the tools you downloaded yesterday are going to work together to make getting your simulation setup really easy. However, we still need to configure them. Lets get into it!

1. Set up Windows Subsystem for Linux (WSL)

In 2010 when I was getting into robotics, it was nearly impossible to start doing Robotics Operating System work without using Linux. But today thanks to WSL we can install an entire operating system as easily as downloading an app. However, there are two versions of WSL, to make sure we have the right one, open an admin terminal in the Windows Terminal app from Day 1 and each line below (hitting enter after it): 

```
wsl.exe --set-version Ubuntu 2
wsl.exe --update
wsl.exe --install
```

(you will be prompted to create a username and password for your linux system. You'll need the username for the next step. If you've already created a username, you'll see it written before an @ symbol. On my computer it looks like john@leatherman , and my username is john )

2. Copy The Course To Your Computer 

In the terminal type `ubuntu` to open an ubuntu shell. 

clone the repo by typing: 

```
git clone https://github.com/johnny555/start-creating-robots-email.git
```

Next we will enter the directory and start vs code.

```
cd start-creating-robots-email
code .
```

You should see vs code start up in your computer. Trust the repo.When VS Code opens up, hit Ctrl+Shift+X to open the extensions sidebar.

Now start docker desktop. (Make sure its using the Ubuntu distro in the background).

3. Open VS Code And Install Dev Container Extension

Search for "Dev Containers" and install the Dev Container extension. (Click the blue install button, or do nothing if its not there because Dev containers is already installed). 

Now press Ctrl+Shift+P and type search for an option "Dev Containers: Open Folder In Container". Make sure docker desktop is running in the background and hit enter. It will ask you to choose a folder. Choose the course folder (that is, the one that is called "start-creating-robot-email")If it asks you about dev container config files, choose "from docker-compose.yaml"

If it asks you to install any extra services - don't, you won't need them. This will now setup and download a working version of ROS straight to your computer! 

4. Run Gazebo

It will take some time to download the docker image, but when it is finished you should see a blue box in the bottom left corner of the VS Code windows which says "Dev Container: Start Creating Robots". 

Press Ctrl+Shift+P and search for Run Task. Next choose "Day 2". 

All going well you should see a gazebo simulation with some balls. Feel free to zoom around and admire your handy-work! Take note of the orange play button, you can use that to start the simulation.

Congrats, you just setup a ROS 2 humble system with a gazebo simulation.

That's it for Day 2. 

# Things To Do For Extra Credit

We are finished for today, but if you feel like it, play around with the gazebo tools. 

After you have hit play the physics simulation will run. You can use the toolbars to spawn in objects, or use the cross-hairs to translate an object and drop it. 

Gazebo also has a lot of plugins, if you click the three dots to the right you will be able to see a list of plugins you can download. Feel free to play with them and see what they do.

You can zoom in and out using your mouses scroll wheel, you can rotate by holding shift while dragging. 

Play with the controls to get a feel for how to move around. 


