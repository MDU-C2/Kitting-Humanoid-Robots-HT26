### Docker Start Script

The `start_container.sh` script provides a convenient and reproducible
way to start the project's ROS 2 Humble and MoveIt 2 development environment.
It automatically checks the required workspace and GUI configuration, 
builds the Docker image when necessary, and starts the development container 
with the ROS 2 workspace mounted at `/workspace`.

The script also configures X11 forwarding, allowing graphical applications
such as **RViz** to run from inside the Docker container.

For implementation details and the complete configuration, 
see [`start.sh`](start.sh).
