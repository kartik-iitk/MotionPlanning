# Motion Planning for RoboCup MSL Robots

This repository includes the Webots simulation for motion planning (local and global) of a RoboCup MSL Robot using a 4-wheel omni-drive.

## Requirements

1. OpenCV v4.8.1 (C++ Version)
2. OMPL v1.6 (C++ Version, python bindings are not required)
2. Webots v2023b
3. cmake

## How to Run

- Install Webots and OpenCV
- Clone the repository and `% cd controllers/omniwheel_supervisor`
- Create a `build` directory and navigate to it: `% mkdir build && cd build`
- Configure using cmake with `% cmake ..`
- Build the project with `% make`
- Open the world in Webots and run the simulation

Note: To use the repository with Visual Studio Code Intellisense, change the necessary include paths in the `.vscode` folder.

## Code Organisation

- omniwheel_supervisor
  - `Coordinate.hpp` => It has definitions of `wheel_angular_vel` struct and `Point2D` struct
  - `Motion.hpp` => Has complete description of class `PID` and the class which implements motion control of the robot.
  - `Visualise.hpp` => All the code necessary for the visualization of the current state of the robot.
  - `Icecream.hpp` => Template Library for Debugging.
  - `PathPlanner.hpp` => Code that needs to be imported in main for generating a set of points.

## Theory

### Wheel Control and Local Motion Planning

- A simple PID controller was integrated with the system to minimize the error.
- Maintaining Orientation:
- Inverse Kinematic Equations:
- Forward Kinematic Equations:
- Control Diagram:

### Global Path Planning

## References

- <https://github.com/MasdikaAliman/Kinematic-4-Omniwheels-in-Webots>
