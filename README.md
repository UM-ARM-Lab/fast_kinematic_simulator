# fast_kinematic_simulator

This package provides the fast kinematic simulator used in our framework for motion planning and execution with actuation uncertainty. More information on the planning and execution methods can be found in our WAFR 2016 [paper](http://arm.eecs.umich.edu/download.php?p=54) and [presentation](https://www.youtube.com/watch?v=42rwqAUTlbo&list=PL24TB_XE22Jvx6Ozhmdwl5kRClbWjUS0m)

## This package provides several core components:

- The core templated kinematic simulator
- Tools for building simulation environments
- Examples for using the simulator directly

While the simulator itself is template-based, this package provides a library containing concrete instantiations of the simulator for different types of robot. When possible, you should use these rather than instantiating the simulator directly.

## Dependencies

- [arc_utilities](https://github.com/UM-ARM-LAB/arc_utilities)
 
Provides a range of utility and math functions.

- [sdf_tools](https://github.com/UM-ARM-LAB/sdf_tools)

Tools for modeling environments using voxel grids, including several types of collision maps, signed distance fields, and optional integration with MoveIt!

- [uncertainty_planning_core](https://github.com/UM-ARM-LAB/uncertainty_planning_core)

Defines the interface for the simulator and provides the robot models.

- [ROS Kinetic](http://ros.org)

ROS is required for the build system, Catkin, and for RViz, which the simulator uses as an optional visualization interface.

## Examples

To see several examples of using the simulator with our planner and execution policy, see [uncertainty_planning_examples](https://github.com/UM-ARM-LAB/uncertainty_planning_examples)
