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

## Managing Your Environment
During the installation of ROS, you will see that you are prompted to source one of several setup.*sh files, or even add this 'sourcing' to your shell startup script. This is required because ROS relies on the notion of combining spaces using the shell environment. This makes developing against different versions of ROS or against different sets of packages easier.

If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup. A good way to check is to ensure that environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set:
```
ros@ros:~$ printenv | grep ROS
```
If they are not then you might need to 'source' some setup.*sh files.

## Create a ROS Workspace
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

#### References
[ETH Zurich - Programming for Robotics - ROS](http://www.rsl.ethz.ch/education-students/lectures/ros.html)

[Youtube - Programming for Robotics (ROS) Course 1](https://www.youtube.com/watch?v=0BxVPCInS3M&list=PLE-BQwvVGf8HOvwXPgtDfWoxd4Cc6ghiP&index=1)

[Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

[Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
