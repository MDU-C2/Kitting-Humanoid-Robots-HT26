# Kitting – Humanoid robots

Project repository for the **DVA490 / DVA474 – Projektkurs i Robotik** course at Mälardalen University (MDU), autumn semester 2026 (HT2026).

Course center: [Projects @ Collaborative Center (C2) MDU](https://github.com/MDU-C2)

## Background
Humanoid robots have demonstrated ability to perform large sweeping motions, however, the software and system architecture required for precise and dexterous industrial task still needs to be developed.

## Purpose
The purpose of the project is to investigate the feasibility of deploying humanoid robots in a industrial environments to perform kitting task. This project should create a system structure for subsequent progress, a foundation for future projects to build on.

## Scope & priorities 
Enabling the kitting task is the top priority.  Identifying, picking and placing the objects in the correct location on the kitting tray is the main task. Enabling safe interaction between the humanoid and its environment, is also encouraged, but not a priority. 

## Main goals and objectives
- Object identification and pose estimation of known objects in clear environment.
- Pick and place procedures in a clear environment. 
- Correct execution of a kitting task in clear lab setting.

## Key reqirements
- The Unitree G1 must be able to pick and place different objects to be kitted without dropping it.
- The Unitree G1 must be able to manipulate the grasped objects for correct placement on the kitting tray.
- The kitting procedure should be completed in a similar timeframe as a human operator.
- The system must use ROS based communication.
- Must not damage itself, other items, structures or living entities.

## Success criteria
The project is a success if the objective “Correct execution of a kitting task in clear lab setting” and all prior main steps, according to the objective priorities, and product requirements are fulfilled

---

## Hardware

| Component | Details |
|---|---|
| Platform | Unitree G1 EDU humanoid |
| Compute | Onboard NVIDIA Jetson Orin |
| Perception | Onboard 3D LiDAR + head-mounted Intel RealSense D435i (RGB-D) |
| Manipulation | Integrated robotic arm |

---

## Required packages
Install the Python dependencies listed in `requirements.txt`:

```bash
pip install -r requirements.txt
```

## ROS 2 Docker Development Environment

The ROS 2 workspace is developed inside a Docker container to provide a
consistent development environment across different computers.

The Docker environment includes ROS 2 Humble, MoveIt 2, ros2_control,
ros2_controllers, and the required development tools. These do not need
to be installed separately on the host system.

### Prerequisites

On a fresh Ubuntu installation, install:

- Git
- Docker
- GitHub access
- A graphical desktop environment for RViz and MoveIt GUI applications

After installing Docker, add your user to the `docker` group so Docker
commands can be executed without `sudo`:

```bash
sudo usermod -aG docker $USER
```

### Setup

Clone the repository:

```bash
git clone https://github.com/MDU-C2/Kitting-Humanoid-Robots-HT26.git
cd Kitting-Humanoid-Robots-HT26
```

Start the development environment with:

```bash
./docker/start.sh
```
