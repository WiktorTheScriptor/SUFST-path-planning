# SUFST-path-planning
path planning module code for formula student driverless 2026/27. Asynchronous pipeline taking in cone maps and vehicle kinematics to output a continuous, kinematically smoothed spatiotemporal centerline trajectory.

## Overview
The Path Planning Module acts as the navigation engine for the car, integrating with the vehicle system as an independent ROS 2 node using a high-speed publish/subscribe model. This subsystem manages three distinct operational states:
- **State 1 (Exploration)** - safely finds and follows the track centerline, allowing the State Estimation/Localisation module to build the map
- **State 2 (Racing)** - executes an optimized, high-speed racing line using the final measured Lap 1 map, which stays fixed
- **State 3 (Recovery)** - drops the vehicle back into a cautious pacing mode if a significant slide or cone collision is detected, allowing it to stabilize before attempting to re-engage the racing line or remap the circuit (similar mode to exploration).

To ensure this repository is maintainable and ready for future speed upgrades, contributors should follow a strict "Write Python, Think C++" architecture, which is described in a section below. This will allow an easier, line-for-line future port to C++.
The ROS2 wrapper unpacks the received data for the submodules, where all core mathematics are kept in classes. There should be strong encapsulation, loose coupling, and good comments/documentation. The code should remain immediately readable and easy to follow when revisiting it after a while.  
Refer to the sections below for specific implementation pipeline specifications, the code styling patterns, and repository folder structures. There will also be recommended TODO tasks in a below section.

## Contribution styling and guidelines
For fast prototyping and getting working features, we use python. However the final code needs to be C++, so writing the python in a way that will be easy to port will save a lot of time and work.
### coding with C++ in mind:
- do not use for loops for matrix maths
  use vectorise NumPy operations which are a lot faster and translate extremely well to C++
- treat python as strongly typed
  use strict type hints and \@dataclass structures (see C++ struct's)
- separate public interfaces and private helper functions for classes
  only expose major entry points, hide 'private' methods with a \_ as the prefix of the name
  (basically keeping track of what methods will be public/private)
- initialise fixed size containers
  avoid using .append() for growing lists, instead pre-allocate a fixed size NumPy array.
  This speeds up the python code, but also translates better to C++ in a way that avoids lag spikes when the cpu needs to allocate another memory block and copy the whole data over.

### Other styling guides:
- data immutability - process 1 takes a 'snapshot' of its input data, performs its calculations, and returns a brand new piece of data. it shouldn't try to modify existing data.
- keep distinct parts of the process (as described in the pipeline specifications) as separate, modular and encapsulated classes/files. It should perform a clear specific task, or be a specific part of the overall pipeline.
- use clear comments and documentation, with explanation why class/method exists, what it does and how it should be used. it is helpful to explain tricky bits in your code. "Always code as if the guy who ends up maintaining your code will be a violent psychopath who knows where you live."
- ROS 2 Wrapper Pattern - for best modularity, the system separates communications from mathematics. The ROS2 node acts strictly as a lightweight "Wrapper Shell" handling subscriptions, data unpacking and publication. The path planning logic exists in standalone Python classes that have zero awareness of ROS2.

## Pipeline architecture
good to use this as a guide/specification for implementing the stages of path planning, ideally as separate encapsulated classes/files.  
there will be 3 state pipelines as mentioned in the overview section, for the exploration, racing and recovery stages/scenarios.
in general, this Path planning module will be taking in vehicle kinematics data (x, y, heading, longitudinal velocity, lateral/sideways velocity, yaw rate) and cone map, which will be a list of cones: (cone id, x, y, colour, covariance matrix, confidence value), which will be provided by the State Estimation / Localisation module.  
This module's output is a spline trajectory section of the planned path (typically 1.5s ahead), as a list of waypoints: (x, y, heading, target velocity, curvature, timestamp), which will be used by the Control module.

### State 1: Exploration
...
### State 2: Racing
...
### State 3: Recovery
...

## Repository Structure
to be decided later, but should have the ROS2 wrapper script top level, with a subdirectory (python package) containing separate python files for each main pipeline step?

## TODO:
- [ ] Finish readme state specifications (focus on states 1 & 2)
- [ ] start implementing state 1 code
- [ ] communicate with State Estimation / Localisation and Control to confirm data formats
- [ ] write some tests for the implemented code/modules
- [ ] communicate with Simulation team how we could test code
