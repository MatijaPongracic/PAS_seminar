# Creating a Simulation Model of CRTA in the Stage Simulator

## Credits
ASTRO & Stage packages have been provided by CRTA_Lab

## Description
Matija Pongračić: Projektiranje autonomnih sustava - seminar

## Requirements
- ROS2 installed on the system
- colcon build tool for building the ROS2 workspace
- Git for cloning the repository
- rosdep for installing dependencies

## Installation and startup
Commands that may need to be run:
```bash
sudo apt update
```
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
```bash
sudo apt install ros-humble-rmw-zenoh-cpp ros-humble-image-transport-plugins
```

### Creating the ROS2 workspace
Run the following commands in the terminal to create the ROS2 workspace and folders for storing experiment results.
```bash
mkdir pas_ws
cd pas_ws
mkdir metrics src
cd metrics
mkdir metrics_robot metrics_sim
cd ~/pas_ws/src
```

### Cloning the repository
Clone the GitHub repository into the `src` folder.
```bash
git clone https://github.com/MatijaPongracic/PAS_seminar.git
```

### Building the project and setting up the environment
Return to the root directory of the workspace, install dependencies, build the workspace and source the setup file.
```bash
cd ~/pas_ws/
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```
NOTE: Make sure to source the setup file in every newly opened terminal!

## (1) Running the system in simulation
To launch the simulation of ASTRO robot navigation on CRTA in the Stage simulator, run:
```bash
ros2 launch astro_bringup sim_bringup.launch.py
```
To open the GUI for selecting the planner and controller and for starting the experiment, run the following command in a second terminal:
```bash
ros2 run astro_navigation nav_experiment_gui.py
```

Use the radio buttons to select the desired planner and controller combination, then press the ENTER button in the GUI.
Then, in RViz, use the 2D Pose Estimate tool to define the robot's initial pose approximately, but as accurately as possible, so that it matches the pose in the Stage simulator.
By selecting 2D Goal Pose in RViz, the robot's goal pose is defined, and if the planner and controller are properly activated, the robot will start navigating.

## (2) Running the system on the real robot
Before starting, open the `bashrc` file using:
```bash
gedit ~/.bashrc
```
Then add the following lines at the end - replace `X` with the number of the ASTRO robot being used (`1-5`):
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=X
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.0.1X:7447"]'
```
Close all terminals and reopen two of them. Check whether the ASTRO robot and the computer are connected to the same network.

NOTE: If the computer is not connected to the robot or the robot is turned off, the added lines in `bashrc` must be removed or commented out!

To launch the navigation system of the real ASTRO robot on CRTA, run:
```bash
ros2 launch astro_bringup robot_bringup.launch.py
```
To open the GUI for selecting the planner and controller and for starting the experiment, run the following command in a second terminal:
```bash
ros2 run astro_navigation robot_nav_experiment_gui.py
```

Use the radio buttons to select the desired planner and controller combination, then press the ENTER button in the GUI.
Then, in RViz, use the 2D Pose Estimate tool to define the robot's initial pose approximately, but as accurately as possible, so that it matches the pose of the real one.
By selecting 2D Goal Pose in RViz, the robot's goal pose is defined, and if the planner and controller are properly activated, the robot will start navigating.

## (3) Conducting the experiment
By default, the experiment is set to be conducted in the Laboratory for Artificial Intelligence (L2), where the robot's initial and goal poses are predefined.

Start the system (simulation or real robot) and the GUI, select the desired planner and controller and press the RESET button. This will assign the initial pose to the robot, which will be visible in RViz. In simulation, this will also place the robot in the equivalent pose in Stage, while in case of working with the real robot, it shall be manually placed in the same pose.

If the robot's initial pose is set correctly, pressing the START button starts the experiment and the results are saved in the appropriate folder - `metrics_sim` or `metrics_robot`.

## (4) Results analysis
### Custom evaluation
After the experiment has been carried out the desired number of times with each planner and controller combination, run the metrics analysis node to compute performance metrics based on the raw odometry data recorded during the experiments.

```bash
ros2 run astro_navigation nav_metrics_analyzer.py --input_dir ~/pas_ws/metrics/metrics_sim/
```
or
```bash
ros2 run astro_navigation nav_metrics_analyzer.py --input_dir ~/pas_ws/metrics/metrics_robot/
```

This will create `csv` files in the folder, which can later be used for performance analysis.

### Evaluation with `evo` tool
Make sure you have installed `evo`. This can be done from PyPI:

```bash
python3 -m pip install --user --upgrade evo
```

Run the evo-analysis node:
```bash
ros2 run astro_navigation evo_analyzer.py --input_dir ~/pas_ws/metrics/metrics_sim/
```
or
```bash
ros2 run astro_navigation evo_analyzer.py --input_dir ~/pas_ws/metrics/metrics_robot/
```

This will create `csv` files in the folder, which can later be used for performance analysis.
