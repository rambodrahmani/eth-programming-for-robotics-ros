# Programming for Robotics (ROS) - Course 1

## Ubuntu install of ROS Kinetic

### Installation
ROS Kinetic ONLY supports Wily (Ubuntu 15.10), Xenial (Ubuntu 16.04) and Jessie (Debian 8) for debian packages. 

### Configure your Ubuntu repositories
Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse."

### Setup your sources.list
Setup your computer to accept software from packages.ros.org.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### Set up your keys
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
If you experience issues connecting to the keyserver, you can try substituting hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 in the previous command. 

### Installation
First, make sure your Debian package index is up-to-date:
```
sudo apt-get update
```
There are many different libraries and tools in ROS. We provided four default configurations to get you started. You can also install ROS packages individually.
Desktop-Full Install: (Recommended) : ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception:
```
sudo apt-get install ros-kinetic-desktop-full
```

To find available packages, use:
```
apt-cache search ros-kinetic
```

### Initialize rosdep
Before you can use ROS, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. 
```
sudo rosdep init
rosdep update
```

### Environment setup
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
*If you have more than one ROS distribution installed, ~/.bashrc must only source the setup.bash for the version you are currently using.*
If you just want to change the environment of your current shell, instead of the above you can type:
```
source /opt/ros/kinetic/setup.bash
```
If you use zsh instead of bash you need to run the following commands to set up your shell:
```
echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

#### References
[ETH Zurich - Programming for Robotics - ROS](http://www.rsl.ethz.ch/education-students/lectures/ros.html)
[Youtube - Programming for Robotics (ROS) Course 1](https://www.youtube.com/watch?v=0BxVPCInS3M&list=PLE-BQwvVGf8HOvwXPgtDfWoxd4Cc6ghiP&index=1)
