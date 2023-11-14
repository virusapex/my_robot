# AutoRace 2023
A template project integrating ROS 2 and Gazebo Garden simulator.

## Included packages

* `robot_description` - holds the SDF description of the simulated system, worlds and any other assets.

* `robot_app` - holds ROS2 specific code and configurations.

* `robot_bringup` - holds launch files and high level utilities.

## Usage

1. Install dependencies

    ```bash
    cd ~/template_ws
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble
    ```

2. Build the project

    ```bash
    colcon build
    ```

3. Source the workspace

    ```bash
    . ~/template_ws/install/setup.sh
    ```

4. Launch the simulation

    ```bash
    ros2 launch robot_bringup autorace_2023.launch.py
    ```
