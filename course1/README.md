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

## Creating a ROS Package
Description: This tutorial covers using roscreate-pkg or catkin to create a new package, and rospack to list package dependencies.

### What makes up a catkin Package?
For a package to be considered a catkin package it must meet a few requirements:
 - The package must contain a catkin compliant package.xml file.
   - That package.xml file provides meta information about the package.
 - The package must contain a CMakeLists.txt which uses catkin.
   - If it is a catkin metapackage it must have the relevant boilerplate CMakeLists.txt file.
 - Each package must have its own folder
   - This means no nested packages nor multiple packages sharing the same directory.

The simplest possible package might have a structure which looks like this: 
```
my_package/
	CMakeLists.txt
	package.xml
```

### Packages in a catkin Workspace
The recommended method of working with catkin packages is using a catkin workspace, but you can also build catkin packages standalone. A trivial workspace might look like this: 
```
workspace_folder/        -- WORKSPACE
	src/                   -- SOURCE SPACEC
		MakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
		package_1/
			CMakeLists.txt     -- CMakeLists.txt file for package_1
			package.xml        -- Package manifest for package_1

		...
		package_n/
			CMakeLists.txt     -- CMakeLists.txt file for package_n
			package.xml        -- Package manifest for package_n
```

### Creating a catkin Package
This tutorial will demonstrate how to use the catkin_create_pkg script to create a new catkin package, and what you can do with it after it has been created.

First change to the source space directory of the catkin workspace you created
```
# You should have created this in the Creating a Workspace Tutorial
$ cd ~/catkin_ws/src
```
Now use the catkin_create_pkg script to create a new package called 'beginner_tutorials' which depends on std_msgs, roscpp, and rospy:
```
ros@ros:~/catkin_ws/src$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
Created file beginner_tutorials/CMakeLists.txt
Created file beginner_tutorials/package.xml
Created folder beginner_tutorials/include/beginner_tutorials
Created folder beginner_tutorials/src
Successfully created files in /home/ros/catkin_ws/src/beginner_tutorials. Please adjust the values in package.xml.
```
This will create a beginner_tutorials folder which contains a package.xml and a CMakeLists.txt, which have been partially filled out with the information you gave catkin_create_pkg.

catkin_create_pkg requires that you give it a package_name and optionally a list of dependencies on which that package depends:
```
# This is an example, do not try to run this
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```
catkin_create_pkg also has more advanced functionalities which are described in catkin/commands/catkin_create_pkg.

### Building a catkin workspace and sourcing the setup file
Now you need to build the packages in the catkin workspace: 
```
ros@ros:~/catkin_ws/src$ cd ~/catkin_ws
ros@ros:~/catkin_ws$ catkin_make
Base path: /home/ros/catkin_ws
Source space: /home/ros/catkin_ws/src
Build space: /home/ros/catkin_ws/build
Devel space: /home/ros/catkin_ws/devel
Install space: /home/ros/catkin_ws/install
####
#### Running command: "cmake /home/ros/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/home/ros/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/home/ros/catkin_ws/install -G Unix Makefiles" in "/home/ros/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/ros/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /opt/ros/kinetic
-- This workspace overlays: /opt/ros/kinetic
-- Using PYTHON_EXECUTABLE: /usr/bin/python
-- Using Debian Python package layout
-- Using empy: /usr/bin/empy
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/ros/catkin_ws/build/test_results
-- Found gmock sources under '/usr/src/gmock': gmock will be built
-- Found gtest sources under '/usr/src/gmock': gtests will be built
-- Using Python nosetests: /usr/bin/nosetests-2.7
-- catkin 0.7.14
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 2 packages in topological order:
-- ~~  - beginner_tutorials
-- ~~  - ros_package_template
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'beginner_tutorials'
-- ==> add_subdirectory(beginner_tutorials)
-- +++ processing catkin package: 'ros_package_template'
-- ==> add_subdirectory(ros_best_practices/ros_package_template)
-- Configuring done
-- Generating done
-- Build files have been written to: /home/ros/catkin_ws/build
####
#### Running command: "make -j2 -l2" in "/home/ros/catkin_ws/build"
####
Scanning dependencies of target sensor_msgs_generate_messages_nodejs
Scanning dependencies of target std_msgs_generate_messages_lisp
[  0%] Built target std_msgs_generate_messages_lisp
[  0%] Built target sensor_msgs_generate_messages_nodejs
Scanning dependencies of target std_msgs_generate_messages_nodejs
Scanning dependencies of target std_msgs_generate_messages_eus
[  0%] Built target std_msgs_generate_messages_nodejs
[  0%] Built target std_msgs_generate_messages_eus
Scanning dependencies of target roscpp_generate_messages_lisp
Scanning dependencies of target roscpp_generate_messages_nodejs
[  0%] Built target roscpp_generate_messages_lisp
[  0%] Built target roscpp_generate_messages_nodejs
Scanning dependencies of target roscpp_generate_messages_eus
Scanning dependencies of target std_msgs_generate_messages_cpp
[  0%] Built target std_msgs_generate_messages_cpp
[  0%] Built target roscpp_generate_messages_eus
Scanning dependencies of target rosgraph_msgs_generate_messages_nodejs
Scanning dependencies of target roscpp_generate_messages_cpp
[  0%] Built target rosgraph_msgs_generate_messages_nodejs
[  0%] Built target roscpp_generate_messages_cpp
Scanning dependencies of target rosgraph_msgs_generate_messages_eus
Scanning dependencies of target roscpp_generate_messages_py
[  0%] Built target rosgraph_msgs_generate_messages_eus
[  0%] Built target roscpp_generate_messages_py
Scanning dependencies of target rosgraph_msgs_generate_messages_cpp
Scanning dependencies of target rosgraph_msgs_generate_messages_lisp
[  0%] Built target rosgraph_msgs_generate_messages_cpp
[  0%] Built target rosgraph_msgs_generate_messages_lisp
Scanning dependencies of target std_msgs_generate_messages_py
[  0%] Built target std_msgs_generate_messages_py
Scanning dependencies of target rosgraph_msgs_generate_messages_py
Scanning dependencies of target geometry_msgs_generate_messages_nodejs
[  0%] Built target rosgraph_msgs_generate_messages_py
[  0%] Built target geometry_msgs_generate_messages_nodejs
Scanning dependencies of target geometry_msgs_generate_messages_lisp
Scanning dependencies of target geometry_msgs_generate_messages_eus
[  0%] Built target geometry_msgs_generate_messages_eus
[  0%] Built target geometry_msgs_generate_messages_lisp
Scanning dependencies of target sensor_msgs_generate_messages_cpp
Scanning dependencies of target geometry_msgs_generate_messages_py
[  0%] Built target geometry_msgs_generate_messages_py
[  0%] Built target sensor_msgs_generate_messages_cpp
Scanning dependencies of target sensor_msgs_generate_messages_py
Scanning dependencies of target sensor_msgs_generate_messages_lisp
[  0%] Built target sensor_msgs_generate_messages_py
[  0%] Built target sensor_msgs_generate_messages_lisp
Scanning dependencies of target geometry_msgs_generate_messages_cpp
Scanning dependencies of target sensor_msgs_generate_messages_eus
[  0%] Built target geometry_msgs_generate_messages_cpp
[  0%] Built target sensor_msgs_generate_messages_eus
Scanning dependencies of target ros_package_template_core
[ 20%] Building CXX object ros_best_practices/ros_package_template/CMakeFiles/ros_package_template_core.dir/src/Algorithm.cpp.o
[ 40%] Linking CXX shared library /home/ros/catkin_ws/devel/lib/libros_package_template_core.so
[ 40%] Built target ros_package_template_core
Scanning dependencies of target ros_package_template
[ 60%] Building CXX object ros_best_practices/ros_package_template/CMakeFiles/ros_package_template.dir/src/ros_package_template_node.cpp.o
[ 80%] Building CXX object ros_best_practices/ros_package_template/CMakeFiles/ros_package_template.dir/src/RosPackageTemplate.cpp.o
[100%] Linking CXX executable /home/ros/catkin_ws/devel/lib/ros_package_template/ros_package_template
[100%] Built target ros_package_template
```
After the workspace has been built it has created a similar structure in the devel subfolder as you usually find under /opt/ros/$ROSDISTRO_NAME.

To add the workspace to your ROS environment you need to source the generated setup file:
```
ros@ros:~/catkin_ws$ source ~/catkin_ws/devel/setup.bash
```
### package dependencies
#### First-order dependencies
When using catkin_create_pkg earlier, a few package dependencies were provided. These first-order dependencies can now be reviewed with the rospack tool.
```
ros@ros:~/catkin_ws$ rospack depends1 beginner_tutorials
roscpp
rospy
std_msgs
```
As you can see, rospack lists the same dependencies that were used as arguments when running catkin_create_pkg. These dependencies for a package are stored in the package.xml file:
```
ros@ros:~/catkin_ws$ roscd beginner_tutorials
ros@ros:~/catkin_ws/src/beginner_tutorials$ cat package.xml 
<?xml version="1.0"?>
<package format="2">
  <name>beginner_tutorials</name>
  <version>0.0.0</version>
  <description>The beginner_tutorials package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="ros@todo.todo">ros</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/beginner_tutorials</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```

#### Indirect dependencies
In many cases, a dependency will also have its own dependencies. For instance, rospy has other dependencies.
```
ros@ros:~/catkin_ws/src/beginner_tutorials$ rospack depends1 rospy
genpy
roscpp
rosgraph
rosgraph_msgs
roslib
std_msgs
```
A package can have quite a few indirect dependencies. Luckily rospack can recursively determine all nested dependencies.
```
ros@ros:~/catkin_ws/src/beginner_tutorials$ rospack depends beginner_tutorials
cpp_common
rostime
roscpp_traits
roscpp_serialization
catkin
genmsg
genpy
message_runtime
gencpp
geneus
gennodejs
genlisp
message_generation
rosbuild
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
ros_environment
rospack
roslib
rospy
```

### Customizing Your Package
This part of the tutorial will look at each file generated by catkin_create_pkg and describe, line by line, each component of those files and how you can customize them for your package.

#### Customizing the package.xml
The generated package.xml should be in your new package. Now lets go through the new package.xml and touch up any elements that need your attention.

##### description tag
First update the description tag:
```
<description>The beginner_tutorials package</description>
```
Change the description to anything you like, but by convention the first sentence should be short while covering the scope of the package. If it is hard to describe the package in a single sentence then it might need to be broken up.

##### maintainer tags
Next comes the maintainer tag:
```
<!-- One maintainer tag required, multiple allowed, one person per tag --> 
<!-- Example:  -->
<!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
<maintainer email="user@todo.todo">user</maintainer>
```
This is a required and important tag for the package.xml because it lets others know who to contact about the package. At least one maintainer is required, but you can have many if you like. The name of the maintainer goes into the body of the tag, but there is also an email attribute that should be filled out:
```
<maintainer email="you@yourdomain.tld">Your Name</maintainer>
```

##### license tags
Next is the license tag, which is also required:
```
<!-- One license tag required, multiple allowed, one license per tag -->
<!-- Commonly used license strings: -->
<!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
<license>TODO</license>
```
You should choose a license and fill it in here. Some common open source licenses are BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, and LGPLv3. You can read about several of these at the Open Source Initiative.
For this tutorial we'll use the BSD license because the rest of the core ROS components use it already:
```
<license>BSD</license>
```

##### dependencies tags
The next set of tags describe the dependencies of your package. The dependencies are split into build_depend, buildtool_depend, exec_depend, test_depend. Since we passed std_msgs, roscpp, and rospy as arguments to catkin_create_pkg, the dependencies will look like this:
```
<!-- The *_depend tags are used to specify dependencies -->
<!-- Dependencies can be catkin packages or system dependencies -->
<!-- Examples: -->
<!-- Use build_depend for packages you need at compile time: -->
<!--   <build_depend>genmsg</build_depend> -->
<!-- Use buildtool_depend for build tool packages: -->
<!--   <buildtool_depend>catkin</buildtool_depend> -->
<!-- Use exec_depend for packages you need at runtime: -->
<!--   <exec_depend>python-yaml</exec_depend> -->
<!-- Use test_depend for packages you need only for testing: -->
<!--   <test_depend>gtest</test_depend> -->
<buildtool_depend>catkin</buildtool_depend>
<build_depend>roscpp</build_depend>
<build_depend>rospy</build_depend>
<build_depend>std_msgs</build_depend>
```
All of our listed dependencies have been added as a build_depend for us, in addition to the default buildtool_depend on catkin. In this case we want all of our specified dependencies to be available at build and run time, so we'll add a exec_depend tag for each of them as well:
```
<buildtool_depend>catkin</buildtool_depend>

<build_depend>roscpp</build_depend>
<build_depend>rospy</build_depend>
<build_depend>std_msgs</build_depend>

<exec_depend>roscpp</exec_depend>
<exec_depend>rospy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

##### Build, Run, and Test Dependencies
Packages can have four types of dependencies:
 - Build Tool Dependencies specify build system tools which this package needs to build itself. Typically the only build tool needed is catkin. In a cross-compilation scenario build tool dependencies are for the architecture on which the compilation is performed.
 - Build Dependencies specify which packages are needed to build this package. This is the case when any file from these packages is required at build time. This can be including headers from these packages at compilation time, linking against libraries from these packages or requiring any other resource at build time (especially when these packages are find_package()-ed in CMake). In a cross-compilation scenario build dependencies are for the targeted architecture.
 - Run Dependencies specify which packages are needed to run code in this package, or build libraries against this package. This is the case when you depend on shared libraries or transitively include their headers in public headers in this package (especially when these packages are declared as (CATKIN_)DEPENDS in catkin_package() in CMake).
 - Test Dependencies specify only additional dependencies for unit tests. They should never duplicate any dependencies already mentioned as build or run dependencies.

These four types of dependencies are specified using the following respective tags:
 - <buildtool_depend>
 - <build_depend>
 - <run_depend>
 - <test_depend>

## Understanding ROS Nodes
Description: This tutorial introduces ROS graph concepts and discusses the use of roscore, rosnode, and rosrun commandline tools.

### Quick Overview of Graph Concepts
 - Nodes: A node is an executable that uses ROS to communicate with other nodes.
 - Messages: ROS data type used when subscribing or publishing to a topic.
 - Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
 - Master: Name service for ROS (i.e. helps nodes find each other)
 - rosout: ROS equivalent of stdout/stderr
 - roscore: Master + rosout + parameter server (parameter server will be introduced later) 

### Nodes
A node really isn't much more than an executable file within a ROS package. ROS nodes use a ROS client library to communicate with other nodes. Nodes can publish or subscribe to a Topic. Nodes can also provide or use a Service.

### Client Libraries
ROS client libraries allow nodes written in different programming languages to communicate:
 - rospy = python client library
 - roscpp = c++ client library

### roscore
roscore is the first thing you should run when using ROS. 
```
ros@ros:~/catkin_ws$ roscore
... logging to /home/ros/.ros/log/894496fe-1d6c-11e9-94b0-0800279864a6/roslaunch-ros-3962.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ros:34441/
ros_comm version 1.12.14


SUMMARY
========

PARAMETERS
 * /rosdistro: kinetic
 * /rosversion: 1.12.14

NODES

auto-starting new master
process[master]: started with pid [3973]
ROS_MASTER_URI=http://ros:11311/

setting /run_id to 894496fe-1d6c-11e9-94b0-0800279864a6
process[rosout-1]: started with pid [3986]
started core service [/rosout]
```

#### References
[ETH Zurich - Programming for Robotics - ROS](http://www.rsl.ethz.ch/education-students/lectures/ros.html)

[Youtube - Programming for Robotics (ROS) Course 1](https://www.youtube.com/watch?v=0BxVPCInS3M&list=PLE-BQwvVGf8HOvwXPgtDfWoxd4Cc6ghiP&index=1)

[Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

[Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

[Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)

[Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

[Building a ROS Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)

[Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)

[Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
