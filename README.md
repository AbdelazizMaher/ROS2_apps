# ROS 2 Applications Workspace

This repository contains various ROS2 applications, implemented in both C++ and Python, for simulating and controlling robotic behaviors using the Turtlesim package. Below is a detailed description of each folder and its contents.

## Repository Structure

### cpp_pkg_ws
- **Description**: This workspace contains the C++ implementation of the ROS2 applications.
- **Contents**: Initial setup and configuration for the C++ workspace.

### cpp_turtlesim_catch_them_all
- **Description**: Contains the C++ implementation of the "Catch Them All" project using the Turtlesim package.
- **Contents**:
  - `src/`: Source files for the `turtle_controller` and `turtle_spawner` nodes.
  - `CMakeLists.txt`: Build configuration for the package.
  - `package.xml`: Package manifest.

### my_custom_bringup
- **Description**: This package includes launch files for starting various nodes and configurations.
- **Contents**:
  - `launch/`: Custom launch files for the ROS2 applications.

### my_custom_interfaces
- **Description**: Contains custom message and service definitions used in the projects.
- **Contents**:
  - `msg/`: Custom message definitions (`Turtle.msg`, `TurtleArray.msg`).
  - `srv/`: Custom service definitions (`CatchTurtle.srv`).
  - `CMakeLists.txt`: Build configuration for the package.
  - `package.xml`: Package manifest.

### py_pkg_ws
- **Description**: This workspace contains the Python implementation of the ROS2 applications.
- **Contents**: Initial setup and configuration for the Python workspace.

### py_turtlesim_catch_them_all
- **Description**: Contains the Python implementation of the "Catch Them All" project using the Turtlesim package.
- **Contents**:
  - `src/`: Source files for the `turtle_controller` and `turtle_spawner` nodes.
  - `CMakeLists.txt`: Build configuration for the package.
  - `package.xml`: Package manifest.


## Project Descriptions

# Turtlesim "Catch Them All" Project

This project uses the Turtlesim package to simulate a scenario where a master turtle (turtle1) catches other turtles that are spawned randomly on the screen. The project involves three main nodes:

- `turtlesim_node` from the Turtlesim package
- `turtle_controller` node: Custom node to control turtle1
- `turtle_spawner` node: Custom node to spawn and manage turtles

## Catch Them All Demo
https://github.com/user-attachments/assets/c23b8c74-848f-4b18-ae26-d577ac49241e

## Nodes Description

### 1. turtlesim_node
The standard Turtlesim node responsible for simulating the turtles and their environment.

### 2. turtle_controller
A custom node to control the movement of the turtle1. The node subscribes to `/turtle1/pose` to get the position of turtle1 and publishes to `/turtle1/cmd_vel` to control its velocity. The node also subscribes to `/alive_turtles` to get the list of currently alive turtles and targets one to catch.

#### Functionalities:
- Runs a control loop to move turtle1 to a given target point using a simplified Proportional (P) controller.
- Selects a turtle to target from the `/alive_turtles` topic.
- Calls the `/catch_turtle` service to catch a turtle when turtle1 reaches it.

### 3. turtle_spawner
A custom node to spawn turtles randomly on the screen and manage the list of currently alive turtles. The node calls the `/spawn` service to create a new turtle and the `/kill` service to remove a turtle.

#### Functionalities:
- Spawns turtles at random coordinates.
- Publishes the list of alive turtles on the `/alive_turtles` topic.
- Handles the `/catch_turtle` service to catch and remove a turtle.

## Custom Interfaces

### Messages
- `Turtle.msg`: Contains the name and coordinates of a turtle.
- `TurtleArray.msg`: An array of `Turtle` messages to publish the list of alive turtles on the `/alive_turtles` topic.

### Services
- `CatchTurtle.srv`: Contains the name of the turtle to be caught. The client is the `turtle_controller` node and the server is the `turtle_spawner` node.

## Parameters and Launch File

### Parameters

#### `turtle_controller` Node
- `catch_closest_turtle_first`: Boolean parameter to determine if the closest turtle should be targeted first.
- `use_sim_time`: Boolean parameter to use simulation time.

#### `turtle_spawner` Node
- `spawn_frequency`: Frequency at which new turtles are spawned.
- `turtle_name_prefix`: Prefix for the names of spawned turtles.
- `use_sim_time`: Boolean parameter to use simulation time.

### Launch File
A launch file in the `my_robot_bringup` package is used to launch the three nodes along with their parameters.

## Steps to Implement

### Step 1: Create the `turtle_controller` Node
- Subscribe to `/turtle1/pose`.
- Create a control loop to reach a given target (initially arbitrary).
- Publish commands to `/turtle1/cmd_vel`.

### Step 2: Create the `turtle_spawner` Node
- Spawn new turtles at a given rate using a timer.
- Call the `/spawn` service to create turtles.

### Step 3: Manage Alive Turtles in `turtle_spawner`
- Maintain an array of alive turtles (name + coordinates).
- Publish this array on the `/alive_turtles` topic.

### Step 4: Implement the `/catch_turtle` Service
- Handle the service in `turtle_spawner`.
- On receiving a turtle name, call the `/kill` service and update the array of alive turtles.

### Step 5: Improve `turtle_controller` to Select the Closest Turtle
- Modify the control logic to target the closest turtle instead of the first turtle in the array.

### Step 6: Add Parameters and Create a Launch File
- Add necessary parameters to the nodes.
- Create a launch file to start all nodes with their parameters.

## Project Demo Video

[[Turtlesim "Catch Them All" Demo]](https://youtu.be/m8cpwuZiixc)
