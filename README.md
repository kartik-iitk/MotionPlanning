# Motion Planning for RoboCup MSL Robots

This repository includes the Webots simulation for motion planning (local and global) of a RoboCup MSL Robot using a 4-wheel omni-drive. This repository is maintained by team ERA, IITK.

## MacOS Installation

Install brew version of UTM for easily autoupdating
UTM settings: <https://docs.getutm.app/guides/ubuntu/>
Install Ubuntu 22.04 (keep all settings defaiult and no need of hardware acceleration OpenGL as we are mostly going to use terminal only): <https://www.youtube.com/watch?v=zeBo5jK7R8Q>
Install ROS2 Jazzy
Install webots_ros2 package:
<https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-MacOS.html>
Run VSCode locally by SSH onto the machine: <https://github.com/utmapp/UTM/discussions/2465#discussioncomment-6931047>
Use UTM for visualisations or running terminal nodes

## Requirements

1. OpenCV v4.8.1 (C++ Version)
2. OMPL v1.6 (C++ Version, python bindings are not required)
2. Webots v2023b
3. cmake
4. boost

__Note__: Use following instructions for Ubuntu -

```
sudo apt update && sudo apt upgrade
sudo apt install libopencv-dev
sudo apt install libboost-all-dev
sudo apt install libompl-dev ompl-demos
```

For MacOS, you need specific homebrew packages.

## How to Run

- Install Webots and OpenCV
- Clone the repository and `% cd controllers/o1`
- Create a `build` directory and navigate to it: `% mkdir build && cd build`
- Configure using cmake with `% cmake ..`
- Build the project with `% make`
- Open the world in Webots and run the simulation

Note: To use the repository with Visual Studio Code Intellisense, change the necessary include paths in the `.vscode` folder.

## Code Organisation

- o1
  - `Coordinate.hpp` => It has definitions of `wheel_angular_vel` struct and `Point2D` struct
  - `Motion.hpp` => Has complete description of class `PID` and the class which implements motion control of the robot.
  - `Robot.hpp` => Provides code needed for interfacing the desired motion from `Motion.hpp` to actual wheel velocities via forward and inverse kinematics with odometry information.
  - `Visualise.hpp` => All the code necessary for the visualization of the current state of the robot.
  - `Icecream.hpp` => Template Library for Debugging.
  - `PathPlanner.hpp` => Code that needs to be imported in main for generating a set of points. Needs to be split into a corresponding PathPlanner.cpp file.

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
