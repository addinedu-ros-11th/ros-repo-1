# Robot Edge Layer

This directory contains all the ROS 2 packages and code that run on the robot itself (the "Edge"). This corresponds to the **Robot Edge Layer** in the system architecture.

## Packages

-   **navigation_node**: Handles SLAM, localization, and path planning using the Nav2 stack.
-   **perception_node**: Processes sensor data from LiDAR and cameras for obstacle detection.
-   **communication_node**: Manages communication with the server via the ROS Bridge and handles video streaming.

## Project Structure (Summary)

At the repository root, the project is organized as follows:

-   `ai_server/`: AI inference server
-   `main_server/`: Main Flask application server
-   `robot-han/`: Robot ROS 2 packages and configuration (this folder)
-   `Docs/`: Project documentation
-   `scripts/`: Database and utility scripts
-   `requirements.txt`: Python dependencies
-   `README.md`: Project overview
-   `.env.example`: Environment variable template
