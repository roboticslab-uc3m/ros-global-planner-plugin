# ros-global-planner-plugin

This package consists of the creation of a new global planner, initially for a Tiago robot from PAL Robotics with its extended arm. The aim of this new global planner is to use a more realistic footprint of the robot taking into account its arm instead of using a bigger radius in the initial circular footprint, as the _global_planner/GlobalPlanner_ by default does. 


The new global planner in the ros-global-planner-plugin package adheres to the _nav_core::BaseGlobalPlanner_ interface defined in the _nav_core_ package and has been created following this tutorial: 
http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS 

**Creation of global planner plugin:**

Following the tutorial mentioned above, first we have the **class header**(_include/roboticslab_global_planner.hpp_): 

In this package, the global planner will use the namespace _roboticslab_. The _roboticslab::GlobalPlanne_ class has been created and it publicly inherits from _nav_core::BaseGlobalPlanner_ class. 

With the costmaps libraries the plugin will access the _costmap_2d::Costmap2DROS_ and _costmap_2d::Costmap2D_ classes and will get automatically the costmap and some useful functions. 

The method _makePlan_ will get the initial point of the robot and the desired goal and will provide the plan to the _move_base_ global planner. The plan will be calculated with a Breadth First Search algorithm. 

Next, the **class implementation** (_src/roboticslab_global_planner.cpp_):

The most important is to register the _BaseGlobalPlanner_ plugin with the following line of code: 

```cpp 
PLUGINLIB_EXPORT_CLASS(roboticslab::GlobalPlanner, nav_core::BaseGlobalPlanner) 
```

and import the library:

```
cpp #include <pluginlib/class_list_macros.h> 
```

In the cpp inicialization some functions from the _costmap_2d::Costmap2D_  class are used to get the costmap, the resolution, the origin and the size in cells and meters. 

Once the costmap is obtained, it is downscale.  For the downscaling a planning scale of 12 units is used, so that the robot with its arm extended which is approximately 1.2 meters occupies two pixels. After there is a threshold to simplify the costmap and eight new costmaps are created taking into account the footprint o the robot in differents orientations (each 45 degrees). 

In the _make plan_ function, the initial point is obtained in meters in the original map, so it has to be transformed to the new resized map considering the offset in meters of the origin, the resolution and the planning scale. Then, the start and goal floor are checked to arrive at the with the right orientation. 


The BFS algorithm looks for the goal using three movements (relative advance, relative rotate right and left). The BFS algorithm uses the class _Node_ which is created in the _include/roboticslab_plan_node.hpp_ file, it has the position coordinates of the robot (x,y) and the orientation in the z parameter, its identification number and its parent identification number. 

Once the goal is found, the traceback begins and the coordinates calculated in the resized map are transformed to the real map. Then, a new goal is pushed back to the robot plan. 

The **plugin description file** (_roboticslab_global_planner_plugin.xml_): 

It is an XML file that store information about the library, the name and type of plugin. 

**Deploy global planner as a plugin:**

The global planner in the navigation should be changed so the move_base package can use it. For that purpose there are some requirements: 

1. Change the  /move_base/base_global_planner parameter of the move_base package with the new global planner name. The effective way to change it is to look for it in a global_planner.yaml file in the configuration of your robot that should look like this:
```cpp 
base_global_planner: roboticslab/GlobalPlanner
GlobalPlanner:
  allow_unknown: false
  default_tolerance: 0.5
  visualize_potential: true
  use_dijkstra: true
  use_quadratic: true
  use_grid_path: false
  old_navfn_behavior: false
  neutral_cost: 80
  cost_factor: 0.5
  last_plan_if_blocked: true
``` 

2. Compile the new package ros-global-planner-plugin and check a new library has been created in your workspace (_~/catkin_ws/devel/lib_) with the name given in the _roboticslab_global_planner_plugin.xml_ file (_libroboticslab_global_planner_lib_). 


**Using the global planner:**

1. Go to the src directory in your workspace and clone the package:   
```cpp 
git clone git@github.com:roboticslab-uc3m/ros-global-planner-plugin.git
```

2. Compile the package with the instruction: 
```cpp 
catkin build ros-global-planner-plugin
```

3. Change the  _/move_base/base_global_planner_ parameter in your robot configuration as previously explained, run your navigation as usual and check that is has been changed with the instruction: 

```cpp 
rosparam get /move_base/base_global_planner
```


