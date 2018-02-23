# ROSBasics
[Basic Commands of Robot Operating System](https://mymultiverse.github.io/ROSBasics/) 

Start with opening terminal with Ctrl+Alt+T and run:-
```markdown
source ~/.bachrc
```
## ROS Master
It is like a connecting wire between various nodes, it allows to run nodes simultaneously. To begin ROS master:-
```markdown
roscore
```
<!--- You can use the [editor on GitHub](https://github.com/mymultiverse/deepspace/edit/master/index.md) to maintain and preview the content for your website in Markdown files. --->

<!--- Whenever you commit to this repository, GitHub Pages will run [Jekyll](https://jekyllrb.com/) to rebuild the pages in your site, from the content in your Markdown files. --->

### Processes
Nodes are different processes. There are publisher nodes and subscriber nodes for every topic. Topics contain messages. Messages can be commands to perform perticular task.    
```markdown
rosrun #rosrun package-name executable-name
rosnode list
rqt_graph
rostopic list
rostopic echo topic-name
rostopic info topic-name
rosmsg show message-type-name
rostopic pub -r 1 /turtle1/cmd_vel geometry_msgTwist '[0,0,0]' '[0,0,1]'



```
## PX4 installation
```markdown
mkdir -p ~/src
cd ~/src
git clone https://github.com/PX4/Firmware.git
cd Firmware
git submodule update --init --recursive
cd ..
```

## Running UAV Simulation with px4 firmware

```markdown
cd ~/src/Firmware
make posix_sitl_default gazebo 
make posix gazebo_iris_opt_flow
make posix_sitl_default gazebo_standard_vtol
make posix_sitl_default gazebo_tailsitter
pxh> commander takeoff
```

For more details see [Documentation](http://dev.px4.io.s3-website-us-east-1.amazonaws.com/simulation-gazebo.html).


# Gazebo

ROS Gazebo interface [info](https://dev.px4.io/en/simulation/ros_interface.html). 
## Interface with ROS
### Creating ROS workspace
```markdown
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
 Create pakage in workspace  
```markdown
cd src
catkin_create_pkg mavtask mavros // catkin_create_pkg <package_name> [dependency1] [dependency2]
```
  Add task.ccp file containing operations to be performed by mav
```markdown
gedit CMakeLists.txt  adding following lins to the file
add_executable(task task.cpp)
target_link_libraries(task ${catkin_LIBRARIES})

cd ~/catkin_ws
catkin_make

```

### Errors

Having trouble ?

[ImportError: No module named catkin_pkg.packages](https://stackoverflow.com/questions/47992442/importerror-no-module-named-catkin-pkg-packages) 

[ImportError: No module](https://robotics.stackexchange.com/questions/14866/error-during-turtlebot-launch-in-ros) pip install module_name.

[Invoking “cmake” failed](https://robotics.stackexchange.com/questions/15107/invoking-cmake-failed) 

[GeographicLib exception](https://robotics.stackexchange.com/questions/14933/roslaunch-mavros-px4-launch-not-working-properly/15132#15132) 

[catkin_make: catkin_pkg.package not found](https://answers.ros.org/question/281598/catkin_make-catkin_pkgpackage-not-found-anaconda/) 

[catkin_ws/devel/setup.bash: No such file or directory](https://answers.ros.org/question/281599/catkin_wsdevelsetupbash-no-such-file-or-directory/) 

# Putting all together 
1. Start ROS master as mentioned above
2. Start gazebo 
```markdown
cd ~/src/Firmware
make posix_sitl_default gazebo
```
3. Launch px4 with mavros
```markdown
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14540"
```
4. Running ros node for created task
```markdown
cd ~/catkin_ws
source ./devel/setup.bash 
rosrun mavtask task 
```
Demo

[![](https://img.youtube.com/vi/sCxQypo6neU/0.jpg)](https://www.youtube.com/watch?v=sCxQypo6neU)

### Making Multi UAV simulation
[Ref](https://dev.px4.io/en/simulation/multi-vehicle-simulation.html)
[Error](https://github.com/PX4/Firmware/pull/7235)
changes to be made in launch file and rcS startup file 
1. Adding code block for new uav with all differnt communication port | in launch file 
2. rcS file in posix.../SITL folder of firware  change in ID mavlink communication port with differnt datastream port


[further](https://www.allaboutcircuits.com/technical-articles/an-introduction-to-robot-operating-system-ros/) 
