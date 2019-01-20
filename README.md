# ETH Programming for Robotics - ROS

### Abstract:
This course gives an introduction to the Robot Operating System (ROS) including many of the available tools that are commonly used in robotics. With the help of different examples, the course should provide a good starting point for students to work with robots. They learn how to create software including simulation, to interface sensors and actuators, and to integrate control algorithms.

### Objective:
- ROS architecture: Master, nodes, topics, messages, services, parameters and actions
- Console commands: Navigating and analyzing the ROS system and the catkin workspace
- Creating ROS packages: Structure, launch-files, and best practices
- ROS C++ client library (roscpp): Creating your own ROS C++ programs
- Simulating with ROS: Gazebo simulator, robot models (URDF) and simulation environments (SDF)
- Working with visualizations (RViz) and user interface tools (rqt)
- Inside ROS: TF transformation system, time, bags

### Content:
This course consists of a guided tutorial and exercises with increasing level of difficulty when working with an autonomous robot. You learn how to setup such a system from scratch using ROS, how to interface the individual sensors and actuators, and finally how to implement first closed loop control systems.

## The course content has been slightly modified in order additional details and personal notes for each of the arguments presented.

## ROS Kinetic
Throughout this course we will be using ROS Kinetic installed on Ubuntu 16.04.

## catkin
### Overview
catkin is the official build system of ROS and the successor to the original ROS build system, rosbuild. catkin combines CMake macros and Python scripts to provide some functionality on top of CMake's normal workflow. catkin was designed to be more conventional than rosbuild, allowing for better distribution of packages, better cross-compiling support, and better portability. catkin's workflow is very similar to CMake's but adds support for automatic 'find package' infrastructure and building multiple, dependent projects at the same time.

The name catkin comes from the tail-shaped flower cluster found on willow trees -- a reference to Willow Garage where catkin was created. 

#### Installing catkin
Catkin is included by default when ROS is installed. Catkin can also be installed from source or prebuilt packages. Most users will want to use the prebuilt packages, but installing it from source is also quite simple.
```
ros@ros:~$ sudo apt-get install python-catkin-tools
```

#### What is a Build System?
A build system is responsible for generating 'targets' from raw source code that can be used by an end user. These targets may be in the form of libraries, executable programs, generated scripts, exported interfaces (e.g. C++ header files) or anything else that is not static code. In ROS terminology, source code is organized into 'packages' where each package typically consists of one or more targets when built.

Popular build systems that are used widely in software development are GNU Make, GNU Autotools, CMake, and Apache Ant (used mainly for Java). In addition, virtually all integrated development environments (IDEs) such as Qt Creator, Microsoft Visual Studio, and Eclipse add their own build system configuration tools for the respective languages they support. Often the build systems in these IDEs are just front ends for console-based build systems such as Autotools or CMake.

To build targets, the build system needs information such as the locations of tool chain components (e.g. C++ compiler), source code locations, code dependencies, external dependencies, where those dependencies are located, which targets should be built, where targets should be built, and where they should be installed. This is typically expressed in some set of configuration files read by the build system. In an IDE, this information is typically stored as part of the workspace/project meta-information (e.g. Visual C++ project file). With CMake, it is specified in a file typically called 'CMakeLists.txt' and with GNU Make it is within a file typically called 'Makefile'. The build system utilizes this information to process and build source code in the appropriate order to generate targets.

ROS utilizes a custom build system, catkin, that extends CMake to manage dependencies between packages.

#### Why Does ROS Have a Custom Build System?
For development of single software projects, existing tools like Autotools, CMake, and the build systems included with IDEs tend to be sufficient. However, these tools can be difficult to use on their own with large, complex, and/or highly heterogeneous code ecosystems -- mainly because of the sheer number of dependencies, complex code organization, and custom build rules a particular target could have. As these tools are very general and designed to be used by software developers, they also tend to be difficult to use by those without a software development background.

ROS is a very large collection of loosely federated packages. That means lots of independent packages which depend on each other, utilize various programming languages, tools, and code organization conventions. Because of this, the build process for a target in some package may be completely different from the way another target is built. catkin specifically tries to improve development on large sets of related packages in a consistent and conventional way. In other words, both rosbuild and now catkin aim to make building and running ROS code easier by using tools and conventions to simplify the process. Efficiently sharing ROS-based code would be more difficult without it.

### Why catkin? Motivation to move away from rosbuild
Though rosbuild served well as the build system for ROS since its inception, the rapid growth of the ROS codebase as well as years of experience have exposed some drawbacks to its approach which catkin attempts to alleviate. As rosbuild was created in the early days of ROS, it has had to evolve from its original design to support the needs of the ROS community over the years. This has led to suboptimal design decisions, hacks, and unnecessary complexity. These problems motivated the creation of a build system on a new foundation.

#### Portability through Python and Pure CMake
One of the biggest problems with rosbuild is that it is not [easily] portable to all operating systems, especially Microsoft Windows. This is because rosbuild utilizes a mixture of Bash scripts, GNU Make, and CMake to build code. In rosbuild, when we invoke the build system, we have to call custom scripts provided with rosbuild such as rosmake. rosmake is a Bash script that calls make which itself calls CMake which generates yet another makefile and finally invoking make again! Catkin is much more elegant and is invoked by simply invoking CMake.

Catkin is implemented as custom CMake macros along with some Python code. As CMake and Python are portable, catkin is easily portable to any system that supports both Python and CMake. In fact, catkin projects can be used seamlessly with other CMake projects -- when catkin projects are built they also generate export information that allows them to be found with the CMake find_package() function.

#### Decoupling from ROS
One of the philosphies behind ROS is to minimize the number of ROS-specific tools needed to create, manage, and utilize ROS packages and to always try to defer to well-established, widely-used, third-party, open-source tools (e.g. using libtinyxml instead of writing a custom XML parser) instead. Catkin is independent of the ROS ecosystem and can even be used on non-ROS projects. That means catkin lets you easily mix your codebase with non-catkin projects. Catkin adds a lot of features on top of vanilla CMake that make it an appealing development tool even for non-ROS related projects.

#### Out-of-Source Builds
When rosbuild builds a package, it generates the targets and any intermediate files (e.g. object files) within the folder containing the code. This is called an in-source build and is often undesirable as it can potentially pollute your local source tree with generated files that are not part of the baseline. With catkin, you can build your targets to any folder -- even one that is external to your package folder. Building targets outside the source folder this way is known as an out-of-source build.

#### Install Targets and Seamless Release
One of the main advantages of catkin, via CMake, is the ability to specify install targets. After code is built and targets are created, they are located in the folders specified by the user to the build system. At this point, though there are usable targets, they have not been installed to the system. All installation means is that the targets are then copied to an area such as a system folder where they can be used by users. For those who have used GNU Make to build code, this is the difference between make and make install where the former builds the code and the latter copies targets to the installation folder. With rosbuild, there is no ability to install targets. When a ROS distribution is released, the core ROS team has to build all packages and then use specialized scripts to extract out targets and build installable packages (e.g. .deb files) for the target operating system. With catkin, one can simply do a "make install" step and have all targets installed. This makes it not only easier for end users to work with ROS, but also allows the core ROS team to release ROS distributions more smoothly.

In addition, catkin allows users to specify which targets are installable and which are not. In a given package, not all targets may be useful and not all may be intended for exporting. For example, it may not be desirable to install unit tests, libraries used only by unit tests, and optional components. With rosbuild, this is not possible and distribution packages tend to contain unnecessary targets which bloat the distribution.

#### Easier Overlays with Workspace Environment Chaining
The concept of overlays has existed in ROS since its early days. Each overlay is associated with a setup file (i.e. setup.bash, setup.py, or setup.sh) that sets the appropriate environment when sourced.

Catkin keeps the concept of overlays utilizing these environment setup files. However, there is a subtle difference from rosbuild -- when a catkin environment setup file is sourced, it overwrites instead of extending existing environment variables. You may then be asking how chaining can work. Simply put, catkin generates lots of setup files in different contexts that chain multiple environments together with a single setup file. When you build code within your workspace, setup files are generated within the "devel space". When these are sourced, any setup files from other workspaces that were used at build time are automatically sourced. For example, if you source /opt/ros/groovy/setup.bash then build your workspace, sourcing the setup.bash file in the devel space will automatically pull in /opt/ros/groovy/setup.bash. Likewise if you install your workspace, the install space will contain a setup.bash file as well that will overlay the install space on top of any workspaces that were used to build that workspace.

#### References
[ETH Zurich - Programming for Robotics - ROS](http://www.rsl.ethz.ch/education-students/lectures/ros.html)
