# Autonomous Tow Truck Project 🚀

## Overview

Welcome to the Autonomous Tow Truck Project! This project focuses on developing an autonomous tow truck using ROS2 for advanced navigation, control, and automation. 

## Directory Structure

The repository is organized into the following directories:

- **`.github/`**: Contains GitHub-specific configurations, including issue and PR templates.
- **`docs/`**: Documentation for the project, including setup guides and API references.
- **`src/`**: Source code for ROS2 packages, organized by modules:
  - `main/`: Core robotics code including:
    - `slam/`: SLAM packages
    - `control/`: Control packages
    - `path_planning/`: Path Planning packages
    - `low_level_control/`: Low-Level Control packages
    - `hardware/`: Hardware-specific packages
    - `simulation/`: Simulation-related packages
  - `shared_libs/`: Shared libraries and utilities
  - `interfaces/`: Custom ROS2 message, service, and action definitions
- **`config/`**: Centralized configuration files for robots, sensors, and environments.
- **`launch/`**: Launch files for deploying the entire system.
- **`models/`**: 3D models and URDF/SDF files for robots and environments.
- **`test/`**: Test cases organized by unit, integration, and performance tests.
- **`tools/`**: Utility scripts and tools for development, including Docker support.
- **`world/`**: Simulation worlds and maps.
- **`package.xml`**: ROS2 package manifest.
- **`setup.cfg`**: Python setup configuration.
- **`setup.py`**: Python package setup file.
- **`.dockerignore`**: Specifies files to ignore in Docker builds.
- **`.gitignore`**: Specifies files to ignore in Git.
- **`.clang-format`**: Code formatting rules for consistency.
- **`LICENSE`**: License file for the project.
- **`README.md`**: This file.

## Setup Instructions

To get started with the project, follow these steps:

1. **Clone the Repository:**

   ```sh
   git clone https://github.com/munzir/autonomous-tow-truck.git
   cd autonomous-tow-truck
   ```
2. **Set Up the Development Environment:**

   - Ensure you have ROS2 Humble installed on Ubuntu 22. Follow the [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html) if needed.
   - Optionally, set up the development environment using Docker:

     ```sh
     cd tools/docker
     docker build -t autonomous-tow-truck .
     ```

   - Run the development environment:

     ```sh
     docker run -it autonomous-tow-truck
     ```

3. **Install Dependencies:**

   - Install Python dependencies:

     ```sh
     pip install -r requirements.txt
     ```

4. **Build the Project:**

   ```sh
   colcon build
   ```

5. **Source the Setup File:**

   ```sh
   source install/setup.bash
   ```
## Usage

To launch the system or individual components, use the ROS2 launch files located in the `launch/` directory. For example:

```sh
ros2 launch launch/robot.launch.py
```

## Testing

Run the tests using the following commands:

- **Unit Tests:**

  ```sh
  colcon test --packages-select <package_name> --test-result
  ```

- **Integration Tests:**

  ```sh
  colcon test --packages-select <package_name> --test-result
  ```

- **Performance Tests:**

  ```sh
  colcon test --packages-select <package_name> --test-result
  ```

## Contributing

For contributions from the team! Please follow these guidelines:

1. **Fork the Repository:** Click the "Fork" button on GitHub to create your own copy of the repository.
2. **Create a Branch:** Create a new branch for your changes:

   ```sh
   git checkout -b <feature_branch>
   ```

3. **Make Changes:** Implement your changes and commit them:

   ```sh
   git add .
   git commit -m "Describe your changes"
   ```

4. **Push Changes:** Push your changes to your fork:

   ```sh
   git push origin <feature_branch>
   ```

5. **Create a Pull Request:** Please create a pull request from your forked repository to the release branch of the target repository. Do not submit it to the main branch

