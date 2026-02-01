# Turtlebot Semantic Mapping and Navigation

## This is a continuation of school projects, merging semantic mapping and semantic navigation.

### To see the original project repositories, visit:
https://github.com/DavidRoss85/Northeastern_University_CS5335_Team_B_Class_Project

---

## Overview

This repository represents a **continued and expanded version** of my original Northeastern University robotics coursework.  
While the original projects focused on meeting course requirements, this version restructures, refactors, and extends the system into a more **modular, reusable, and scalable robotics software stack**.

The project combines:
- **Semantic perception** (object detection and localization)
- **Semantic mapping** (overlaying detected objects onto occupancy grids)
- **Navigation and planning**
- **Custom navigation logic** designed to address limitations encountered with standard Nav2 behaviors

This version is no longer a class-only submission, but an evolving personal project intended for experimentation, learning, and portfolio demonstration.

---

## What’s New in This Version

### 1. Major Directory & Package Restructure
The codebase has been reorganized to improve:
- Separation of concerns
- Reusability across projects
- Long-term maintainability

Previously intertwined logic has been broken out into clearer modules (e.g., perception, mapping, navigation, shared utilities).  
This makes it easier to test components independently and reuse them in future robots or simulations.

---

### 2. Improved Semantic Mapping Pipeline
The semantic mapping pipeline has been refined to:
- Cleanly ingest detection outputs (e.g., YOLO-based object detections)
- Convert detections into world-frame locations
- Overlay semantic information onto an occupancy grid map
- Apply configurable inflation and filtering for navigation safety

Mapping logic is now more configurable and easier to extend with additional object classes or mapping strategies.

---

### 3. Custom Navigation Module (Beyond Nav2)
During development, limitations were encountered when relying solely on the default Nav2 navigator for higher-level semantic behaviors.

As a result, this project introduces a **custom navigation module** that:
- Wraps or replaces parts of the Nav2 navigation flow
- Enables more explicit control over navigation states
- Supports future integration with semantic goals (e.g., “navigate to object X” rather than just coordinates)

This module is still evolving but serves as a foundation for:
- State-machine–based navigation
- Object-aware navigation goals
- Tighter coupling between perception, mapping, and motion

---

### 4. Clearer Data Flow Between Modules
Compared to the original coursework version, this repository emphasizes:
- Explicit data flow between perception → mapping → navigation
- Cleaner interfaces between ROS2 nodes
- Reduced hidden coupling between components

This makes the system easier to debug, extend, and reason about—especially as complexity grows.

---

## Technologies Used

- **ROS 2 (Jazzy)**
- **Python-based ROS nodes**
- **Nav2 (with custom extensions)**
- **YOLO-based object detection**
- **Occupancy grid mapping**
- **Custom path planning and navigation logic**
- **TurtleBot platform (simulation + real hardware)**

---

## Current Status

This project is **actively evolving**.  
Some components are stable, while others (especially the custom navigation layer) are under active development and experimentation.

The repository reflects a real-world robotics workflow:
- Iterative refactoring
- Debugging around hardware and simulation constraints
- Gradual abstraction of reusable components

---

## Future Directions

Planned or exploratory extensions include:
- Deeper semantic goal reasoning (object-based navigation)
- Integration with mobile manipulation pipelines
- Improved planner heuristics using semantic context
- Further decoupling of robot-specific vs. robot-agnostic logic

---

## Notes

This repository prioritizes **clarity, learning, and extensibility** over being a polished, finished product.  
It is intended to demonstrate how a robotics system evolves beyond coursework into a more realistic engineering project.

Feedback, exploration, and reuse are welcome.
