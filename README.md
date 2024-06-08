# Autonomous Tow Truck Project
## Installation
### Prerequisites
- Ubuntu 20.04 
- ROS (Robot Operating System) Noetic
- Python 3.8+
- OpenCV
### Steps
1. Clone the repository:
    ```sh
    git clone https://github.com/muzir/autonomous-tow-truck.git
    cd autonomous-tow-truck
    ```

2. Install the required dependencies:
    ```sh
    sudo apt update
    sudo apt install ros-noetic-desktop-full python3-opencv pcl-tools
    pip install -r requirements.txt
    ```

3. Build the ROS workspace:
    ```sh
    catkin_make
    source devel/setup.bash
    ```
## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a new branch:
    ```sh
    git checkout -b feature-branch
    ```
3. Commit your changes:
    ```sh
    git commit -am 'Add new feature'
    ```
4. Push to the branch:
    ```sh
    git push origin feature-branch
    ```
5. Create a new Pull Request.

Please ensure your code adheres to the project's coding standards and includes relevant tests.
