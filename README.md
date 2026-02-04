# Turtlebot Semantic Mapping and Navigation

## This is a continuation of school projects, merging semantic mapping and semantic navigation.

### To see the original project repositories, visit:

[https://github.com/DavidRoss85/Northeastern_University_CS5335_Team_B_Class_Project](https://github.com/DavidRoss85/Northeastern_University_CS5335_Team_B_Class_Project)

---

## Overview

This repository represents a **continued and expanded version** of my Northeastern University robotics coursework, focused on integrating **semantic perception, semantic mapping, and navigation** into a cohesive mobile robotics system.

While the original course project emphasized semantic mapping, this version restructures and extends the system to improve modularity, maintainability, and autonomy. The codebase has been refactored to better separate perception, mapping, navigation, and shared utilities, enabling clearer data flow and easier future expansion.

In addition to refining the semantic mapping pipeline, this project introduces a **custom navigation module** developed in response to limitations encountered with the default Nav2 navigator when handling higher-level, semantic-aware behaviors.

The broader motivation for this work is inspired by applications such as **search and rescue**, situational awareness, and information gathering, where semantic understanding of an environment can significantly improve decision-making. Similar concepts extend to domains such as autonomous inspection, investigation, and vehicular navigation systems.

---

## Quick Start / How to Run

The project provides scripts to simplify setup and execution. After installation, the full perception–mapping–navigation pipeline can be launched with a single command.

### Installation Instructions

#### 1. Install System Dependencies (ROS Jazzy + Nav2 + TurtleBot4)

From the project root:

```bash
./scripts/install_binaries.sh
```

This installs:

* ROS 2 Jazzy
* Navigation2
* nav2-bringup
* nav2-minimal-tb*
* nav2-route
* TurtleBot4 packages
* teleop-twist-keyboard
* rosbridge-server
* Gazebo simulator for turtlebot

---

#### 2. Set Up Virtual Environment and Python Dependencies

```bash
./scripts/setup_env.sh
```

This script:

* Creates a `venv/` directory
* Installs dependencies from `requirements.txt`
* Installs additional packages, including:

  * typeguard
  * ultralytics
  * opencv-python
  * scipy
  * message-filters
* Opens a new terminal and performs a clean `colcon build` (without activating the virtual environment)

---

### Launching the System

After completing installation and environment setup, launch the full system from the project root:

For live Demo: (Current Live demo script is being updated. Use Gazebo simulation for now...):
<!-- ```bash -->
<!-- ./scripts/run_live_demo.sh -->
<!-- ``` -->
For Simulation in Gazebo Demo:
```bash
./scripts/run_simulated_demo.sh
```

This script automatically launches:

* Perception pipeline (YOLO-based object detection + depth processing)
* SLAM or map loading (depending on configuration)
* Nav2 navigation stack
* Custom approach controller
* Required supporting nodes

No additional manual launch steps are required.

---

### Using the System After Launch

Once the live demo script is running, the full perception–navigation–approach pipeline initializes automatically.

At this stage:

* Object detections are processed and projected into the world frame
* Semantic information is overlaid onto the occupancy grid
* Navigation components receive updated map and state information
* The system is ready for operator interaction or autonomous behaviors, depending on configuration
* Clicking an explored location on the map vizualizer will navigate the Turtlebot to that point (experimental)

Further runtime interaction and parameter tuning can be performed through ROS tools (e.g., RViz, CLI commands, or custom interfaces), depending on the experiment or demonstration goals.

---

## What’s New in This Version

### 1. Modular Codebase Restructure

The project has been reorganized to improve:

* Separation of concerns between perception, mapping, and navigation
* Reusability of shared utilities and geometry helpers
* Long-term maintainability beyond a single course timeline

Previously intertwined logic has been split into clearer, self-contained modules.

---

### 2. Refined Semantic Mapping Pipeline

The semantic mapping system has been improved to:

* Ingest object detections from a perception pipeline
* Estimate object positions in the world frame
* Overlay semantic information onto an occupancy grid
* Apply configurable inflation and filtering for navigation safety

The pipeline is now more configurable and easier to extend with additional object classes or mapping strategies.

---

### 3. Custom Navigation Module

During development, limitations were encountered when relying solely on the default Nav2 navigation stack for semantic-driven behaviors.

To address this, a **custom navigation module** was introduced to:

* Provide more explicit control over navigation states
* Enable future semantic-goal navigation (e.g., object-aware goals)
* Improve integration between perception, mapping, and motion planning

This module serves as a foundation for higher-level autonomy and coordinated behaviors.

---

## Demo Video

A short demonstration of the system running in a Gazebo simulation is available here:

* **Semantic Mapping and Navigation Demo (Simulation)**
  Demonstrates object detection, semantic map generation, and navigation behavior within a simulated environment.
  🔗 [https://drive.google.com/file/d/1f5Ts-ItUsDT_y8hnMnnXCJmgK7GD9nkV/view?usp=sharing](https://drive.google.com/file/d/1f5Ts-ItUsDT_y8hnMnnXCJmgK7GD9nkV/view?usp=sharing)

> The demo is shown in simulation to clearly illustrate system behavior and data flow without hardware-specific constraints.

---

## Development Considerations & Limitations

As this project was developed and tested using both simulation and real TurtleBot hardware, several real-world constraints influenced design decisions:

* **Network latency and bandwidth limitations** impacted perception and control loops, particularly when streaming RGB-D data
* **Camera and sensor configuration** required iterative tuning due to limited or fragmented hardware documentation
* Some parameters (e.g., depth alignment, frame rates, resolution tradeoffs) were refined through empirical testing rather than prescriptive guides
* These constraints motivated clearer module boundaries and increased configurability across system components

While not all edge cases are fully resolved, the system reflects realistic tradeoffs encountered in deployed robotic systems.

---

## Technologies Used

* **ROS 2 (Jazzy)**
* **Python-based ROS nodes**
* **Nav2 (with custom extensions)**
* **YOLO-based object detection**
* **Occupancy grid mapping**
* **Semantic map overlays**
* **Gazebo simulation**
* **TurtleBot platform (simulation and real hardware)**

---

## Future Directions

Planned and exploratory extensions include:

* Enabling the robot to **populate a shared semantic map** containing multiple detected objects and their world-frame locations
* Allowing a human operator or autonomous system to **select semantic targets** directly from the map
* Broadcasting selected targets to **other mobile robots**, enabling coordinated or distributed navigation tasks
* Supporting higher-level mission logic such as task assignment, prioritization, and multi-robot collaboration

These directions aim to move the system toward scalable, semantic-aware multi-robot autonomy.

---

## Notes

This repository prioritizes **clarity, extensibility, and learning** over being a finalized production system.
It represents an evolving robotics project that builds on academic foundations while addressing real-world constraints and design tradeoffs.
