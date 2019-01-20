# Programming for Robotics (ROS) - Course 1

## Ubuntu install of ROS Kinetic

### Installation
ROS Kinetic ONLY supports Wily (Ubuntu 15.10), Xenial (Ubuntu 16.04) and Jessie (Debian 8) for debian packages. 

### Configure your Ubuntu repositories
Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse."

### Setup your sources.list
Setup your computer to accept software from packages.ros.org.
```
ros@ros:~$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### Set up your keys
```
ros@ros:~$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
If you experience issues connecting to the keyserver, you can try substituting hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 in the previous command. 

### Installation
First, make sure your Debian package index is up-to-date:
```
ros@ros:~$ sudo apt-get update
```
There are many different libraries and tools in ROS. We provided four default configurations to get you started. You can also install ROS packages individually.

Desktop-Full Install: (Recommended) : ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception:
```
ros@ros:~$ sudo apt-get install ros-kinetic-desktop-full
```

To find available packages, use:
```
ros@ros:~$ apt-cache search ros-kinetic
```

### Initialize rosdep
Before you can use ROS, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. 
```
ros@ros:~$ sudo rosdep init
ros@ros:~$ rosdep update
```

### Environment setup
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
```
ros@ros:~$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
ros@ros:~$ source ~/.bashrc
```
*If you have more than one ROS distribution installed, ~/.bashrc must only source the setup.bash for the version you are currently using.*
If you just want to change the environment of your current shell, instead of the above you can type:
```
ros@ros:~$ source /opt/ros/kinetic/setup.bash
```
If you use zsh instead of bash you need to run the following commands to set up your shell:
```
ros@ros:~$ echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
ros@ros:~$ source ~/.zshrc
```

## Installing and Configuring Your ROS Environment
### Managing Your Environment
During the installation of ROS, you will see that you are prompted to source one of several setup.*sh files, or even add this 'sourcing' to your shell startup script. This is required because ROS relies on the notion of combining spaces using the shell environment. This makes developing against different versions of ROS or against different sets of packages easier.

If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup. A good way to check is to ensure that environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set:
```
ros@ros:~$ printenv | grep ROS
```
If they are not then you might need to 'source' some setup.*sh files.

### Create a ROS Workspace
Let's create and build a catkin workspace:
```
ros@ros:~$ mkdir -p ~/catkin_ws/src
ros@ros:~$ cd ~/catkin_ws/
ros@ros:~$ catkin_make
```
The catkin_make command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a CMakeLists.txt link in your 'src' folder. Additionally, if you look in your current directory you should now have a 'build' and 'devel' folder. Inside the 'devel' folder you can see that there are now several setup.*sh files. Sourcing any of these files will overlay this workspace on top of your environment.
```
ros@ros:~/catkin_ws$ ls -l
total 12
drwxrwxr-x 7 ros ros 4096 gen 20 12:59 build
drwxrwxr-x 3 ros ros 4096 gen 20 12:58 devel
drwxrwxr-x 2 ros ros 4096 gen 20 12:58 src
```
Before continuing source your new setup.*sh file: 
```
ros@ros:~/catkin_ws$ source devel/setup.bash
```
To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.
```
ros@ros:~/catkin_ws$ echo $ROS_PACKAGE_PATH
/home/ros/catkin_ws/src:/opt/ros/kinetic/share
```
Now your environment is setup.

## Navigating the ROS Filesystem
Description: This tutorial introduces ROS filesystem concepts, and covers using the roscd, rosls, and rospack commandline tools.

### Prerequisites
For this tutorial we will inspect a package in ros-tutorials, please install it using
```
ros@ros:~/catkin_ws$ sudo apt-get install ros-kinetic-ros-tutorials
[sudo] password for ros: 
Reading package lists... Done
Building dependency tree       
Reading state information... Done
ros-kinetic-ros-tutorials is already the newest version (0.7.1-0xenial-20181107-045510-0800).
ros-kinetic-ros-tutorials set to manually installed.
0 upgraded, 0 newly installed, 0 to remove and 0 not upgraded.
```

### Quick Overview of Filesystem Concepts
 - Packages: Packages are the software organization unit of ROS code. Each package can contain libraries, executables, scripts, or other artifacts.
 - Manifests (package.xml): A manifest is a description of a package. It serves to define dependencies between packages and to capture meta information about the package like version, maintainer, license, etc...

### Filesystem Tools
Code is spread across many ROS packages. Navigating with command-line tools such as ls and cd can be very tedious which is why ROS provides tools to help you.

#### Using rospack
rospack allows you to get information about packages. In this tutorial, we are only going to cover the find option, which returns the path to package.

Usage:
```
$ rospack find [package_name]
```
Example:
```
ros@ros:~/catkin_ws$ rospack find roscpp
/opt/ros/kinetic/share/roscpp
```
#### Using roscd
roscd is part of the rosbash suite. It allows you to change directory (cd) directly to a package or a stack.

Usage:
```
$ roscd [locationname[/subdir]]
```
To verify that we have changed to the roscpp package directory, run this example:
```
ros@ros:~/catkin_ws$ roscd roscpp
ros@ros:/opt/ros/kinetic/share/roscpp$ pwd
/opt/ros/kinetic/share/roscpp
````
You can see that YOUR_INSTALL_PATH/share/roscpp is the same path that rospack find gave in the previous example.

Note that roscd, like other ROS tools, will only find ROS packages that are within the directories listed in your ROS_PACKAGE_PATH. To see what is in your ROS_PACKAGE_PATH, type:
```
ros@ros:~/catkin_ws$ echo $ROS_PACKAGE_PATH
/home/ros/catkin_ws/src:/opt/ros/kinetic/share
```
Your ROS_PACKAGE_PATH should contain a list of directories where you have ROS packages separated by colons. A typical ROS_PACKAGE_PATH might look like this:
```
/opt/ros/kinetic/base/install/share
```
Similarly to other environment paths, you can add additional directories to your ROS_PACKAGE_PATH, with each path separated by a colon ':'.

##### Subdirectories
roscd can also move to a subdirectory of a package or stack.

Try:
```
ros@ros:/opt/ros/kinetic/share/roscpp$ roscd roscpp/cmake
ros@ros:/opt/ros/kinetic/share/roscpp/cmake$ pwd
/opt/ros/kinetic/share/roscpp/cmake
```
#### roscd log
roscd log will take you to the folder where ROS stores log files. Note that if you have not run any ROS programs yet, this will yield an error saying that it does not yet exist.
If you have run some ROS program before, try:
```
ros@ros:/opt/ros/kinetic/share/roscpp/cmake$ roscd log
No active roscore
bash: cd: /home/ros/.ros/log: No such file or directory
```
#### Using rosls
rosls is part of the rosbash suite. It allows you to ls directly in a package by name rather than by absolute path.

Usage:
```
$ rosls [locationname[/subdir]]
```
Example:
```
ros@ros:/opt/ros/kinetic/share/roscpp/cmake$ rosls roscpp_tutorials
cmake  launch  package.xml  srv
```
#### Tab Completion
It can get tedious to type out an entire package name. In the previous example, roscpp_tutorials is a fairly long name.

Luckily, some ROS tools support TAB completion.

Start by typing:
```
$ roscd roscpp_tut<<< now push the TAB key >>>
```
After pushing the TAB key, the command line should fill out the rest:
```
ros@ros:/opt/ros/kinetic/share/roscpp/cmake$ roscd roscpp_tutorials/
ros@ros:/opt/ros/kinetic/share/roscpp_tutorials$ pwd
/opt/ros/kinetic/share/roscpp_tutorials
```
This works because roscpp_tutorials is currently the only ROS package that starts with roscpp_tut.
Now try typing:
```
$ roscd tur<<< now push the TAB key >>>
```
After pushing the TAB key, the command line should fill out as much as possible:
```
$ roscd turtle
```
However, in this case there are multiple packages that begin with turtle. Try typing TAB another time. This should display all the ROS packages that begin with turtle:
```
ros@ros:/opt/ros/kinetic/share/roscpp_tutorials$ roscd turtle
turtle_actionlib/  turtlesim/         turtle_tf/         turtle_tf2/        
ros@ros:/opt/ros/kinetic/share/roscpp_tutorials$ roscd turtle
```

#### References
[ETH Zurich - Programming for Robotics - ROS](http://www.rsl.ethz.ch/education-students/lectures/ros.html)

[Youtube - Programming for Robotics (ROS) Course 1](https://www.youtube.com/watch?v=0BxVPCInS3M&list=PLE-BQwvVGf8HOvwXPgtDfWoxd4Cc6ghiP&index=1)

[Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

[Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

[Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
