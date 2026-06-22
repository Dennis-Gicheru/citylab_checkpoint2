Citylab_Project: Autonomous Patrol and Navigation System


Overview
This repository contains an autonomous navigation and patrol framework designed for both simulated (Gazebo) and real-world environments. Built on the ROS 2 Humble distribution, the system emphasizes a highly modular, scalable, and maintainable C++ architecture.

The core of the project relies on robust service-client interactions and action servers to manage complex robotic states, ensuring precise point-to-point navigation and continuous service-driven patrolling.

Features
Service-Driven Patrol Logic: A dedicated patrol_with_service node that handles continuous operational routines decoupled from basic movement commands.

Action Server Integration: Implementation of a go_to_pose action server for robust, interruptible, and feedback-oriented goal navigation.

Custom ROS 2 Interfaces: Domain-specific custom .srv and .action definitions to strictly type the data passed between nodes.

Seamless Simulation to Reality Transfer: Configured to map navigation topics dynamically, allowing identical software execution in Gazebo simulations and physical hardware deployment.

Advanced Visualization: Customized RViz configurations tailored for real-time monitoring of the robot's model, odometry, and navigation goals.

System Architecture
To adhere to industry-standard software engineering practices, the functionality is heavily modularized into distinct, single-responsibility components:

1. Custom Interfaces
GoToPose.action: Defines the goal pose, continuous distance-to-goal feedback, and final success state for the action server.

GetDirection.srv: A synchronous service definition used by the direction server to compute and provide immediate vector or pathing data to clients.

2. Core Nodes
patrol.cpp: The foundational autonomous movement logic, heavily optimized for stability and accurate robot model representation.

Direction Service Server & Client: A decoupled request-response architecture allowing the robot to query optimal directions dynamically.

Go To Pose Action Server: Manages long-running navigation tasks, interfacing directly with the navigation stack while providing continuous status updates.
