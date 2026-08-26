# ROS Trajectory Simulator for UR10e

## Table of Contents
**1. Current Features**                     <-- Current capability of the repo

**2. Features In Progress/Current goals**   <-- Features that still need to be implmented

**3. Set Up and Running Guide**             <-- How to set up and run the code

**4. File Structure and notes to user**     <-- Important notes about assumptions in a few files for the user to note

**5. Key notation and Frame definitions**


## Current Features
**<u>ROS Communication</u>**

* ROS Server: `Validate_trajServer.py`
* ROS Client: `ClientNode.py`

**<u>Works directly from SISIFOS </u>**

* Reads in a .csv of spacecraft coordinates relative to a target (ASTROS) and transforms them into end effector coordinates relative to the base of the UR10e robot arm

**<u>Finds safe segments</u>**

* Checks each waypoint for:
    - Large joint accelerations (default 2.0 rad/s)
        - Threshold can be set in validation_core's find_feasible_segments() function
    - Singularities
        - Uses numerical condition number to verify controlabillity (default threshold is 50.0)
            - Threshold can be set in validation_core's find_feasible_segments() function
    - Collisions
        - Uses pybullet to check for self-instersections and collisions with the wall and/or floor
* If the full trajectory is not safe, `validation_core.py` finds all the safe segments
    * `Validate_trajServer.py` includes a hyperparameter for the number of safe way points in a row before a segment is reported.


**<u>RViz Simulation </u>**

* Displays the UR10e arm and rail with the lab's wall (in gold) and wall (in dark grey) as a validated trajectory is performed
    * `VisualizeTraj_RvizPlayback.py`
    * `obstacle_markers.py`

**<u>Base translation </u>**
* Includes the rail as a 7th degree of freedom in all IK optimizations


## Features In progress

**<u>Connecting to Hardware </u>**
* Reading in true relative locations from VICON
* Setup external driver to run the UR10e after a trajectory is validated

**<u>Joint velocity checking on the rail </u>**
* Current code lacks a way to limit rail velocities that would produce non-feasible/unsafe rail commands

**<u>Orientation only simulation </u>**
* Update ClientNode.py to clearly define each frame (with the A frame equal to the G frame for ow)
* As an immediate goal want hold the translation of the A/G frame constant and simulate pure tumbling motion based on the quaterions



## Set Up and Running Guide 
#### In terminal #1
1) Clone the workspace (ros2_ws) from GitHub
2) Build the docker image from the DockerFile

    - `cd ~/ros2_ws`

    - `docker build -t ur10e_sim .`

3) Mount the workspace to the docker image (ur10e_sim)

    - Enable Rviz to run properly: `xhost +local:docker`
    - Mount: `docker run -it --rm \
    --name ur10e_sim \
    -v ~/ros2_ws:/root/ros2_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix
    -e DISPLAY=$DISPLAY
ur10e_sim bash`

`
4) Build and source the packages

- Clear old cache: `rm -rf build install log` 
- Build: `colcon build`
- Source: `source install/setup.bash`


5) Run the validation service

    - `ros2 run ur10e_trajectory_pkg validation_server`


#### In terminal #2
1) Enter the docker container from Terminal #1
    - `docker exec -it ur10e_sim bash`
2) Create a node to publish visualize the obstacles
    - `ros2 run ur10e_trajectory_pkg obstacle_markers`


#### In terminal #3

1) Enter the docker container from Terminal #1
    - `docker exec -it ur10e_sim bash`
2) Launch Rviz and connect to the /obstacle_markers and /joint_states topics
    - `ros2 launch ur10e_trajectory_pkg VisualizeTraj_RvizPlayback.py`


#### In terminal #4

1) Enter the docker container from Terminal #1
    - `docker exec -it ur10e_sim bash`
2) Run the trajectory creation client to convert from SISIFOS to Arena coords and send a request to the server
    - `ros2 run ur10e_trajectory_pkg trajectory_client`



## File Structure and notes to user
### Project root
    ros2_ws
        ├── ur10e.urdf
        ├── test_validation_local.py      <-- Rviz doesn't work from ssh, this file can be used to test 
                                            code locally (Replaces ClientNode.py and Validate_trajServer.py)
        ├── validation_core.py            <-- Called by test_validation_local.py to validate the trajectory
        ├── smoke_test.py                 <-- Use this to examine collisions reported from test_validation_local.py
        ├── VisualizeContact.py           <-- Use this to examine a configuration and contact point from smoke_test.py
        ├── contact_scene.png             <-- Example output of VisualizeContact.py
        ├── src                           <-- Directory which contains the ROS architecture
        └── Dockerfile                    <-- Builds a docker image with all the needed dependancies to 
                                                mount the Ros2 workspace on. Is also compatible with Gazebo (for future controls work)
    ros2_ws/src
        ├── Universal_Robots_ROS2_Description <--Official Universal Robots package 
                                                (for meshes and collision geometry)
        ├── ur10e_interfaces  <--Interfaces Package
        └── ur10e_trajectory_pkg<--Main Package

### Interfaces Package
    ros2_ws/src/ur10e_interfaces/srv
        └── ValidateTrajectory.srv

### Main Package

    ros2_ws/src/ur10e_trajectory_pkg/ur10e_trajectory_pkg
        ├── ClientNode.py                   <-- Note that there is dummy code with a known 
                                            feasible trajectory (not from SISIFOS) for testing purposes. 
                                            Also the scale factor and the values of p_B_I and q_I_B are placeholders. 
                                            The real values will come from VICON.

        ├── Validate_trajServer.py          <-- Explaination of parameters below:
            SKIP_COLLISION=True turns off collision checking for debugging purposes. 
            USE_SEGMENT_FINDER=True turns on the ability to find feasible segments in a larger trajectory.
            MIN_SEGMENT_LENGTH is the minimum number of safe waypoints before a segment is reported
            DEFAULT_T_TRAJ, the timestep for the validation is computed as DEFAULT_T_TRAJ / (num_waypts-1)

        ├── VisualizeTraj_RvizPlayback.py   
        ├── __init__.py
        ├── build
        ├── camera_traj.csv                 <--Must ensure that you replace this with any new trajectories
                                            you wish to check
        ├── install
        ├── log
        ├── obstacle_markers.py              
        ├── old.zip                          <-- Old code for reference/debugging
        └── validation_core.py               

## Key Notation and Frame Definitions

For two points $X,Y\in\mathbb{E}^3$, we define the translation vector
$$\vec r_{YX} \;\triangleq\; Y - X \in \mathbb{R}^3,$$
which is the vector from $X$ to $Y$. A superscript indicates the expressing frame, e.g., $\vec r_{YX}^{\mathcal O}$.

Rotations are indexed as $R_{XY}\in\mathrm{SO}(3)$ and map coordinates from frame $\mathcal Y$ to frame $\mathcal X$, expressed as
$$v^{\mathcal X} = R_{XY}\,v^{\mathcal Y}.$$
The inverse is $R_{YX}=R_{XY}^\top$. Rotation composition follows
$$R_{XZ} = R_{XY}\,R_{YZ},$$
which corresponds to applying $R_{YZ}$ first, then $R_{XY}$.

A pose is denoted as $T_{XY} = (R_{XY},\,\vec r_{YX}^{\mathcal X}) \in \mathrm{SE}(3)$. Under pose composition $T_{XZ}=T_{XY}\circ T_{YZ}$, translations compose as
$$\vec r_{ZX}^{\mathcal X} = \vec r_{YX}^{\mathcal X} + R_{XY}\,\vec r_{ZY}^{\mathcal Y}.$$

We define a right-trivialized retraction (“$\boxplus$”) on $\mathrm{SO}(3)$ and $\mathrm{SE}(3)$ by
$$R \boxplus \delta\theta \triangleq R\,\exp\!\big((\delta\theta)^\wedge\big),$$
$$T \boxplus \delta\xi \triangleq T\,\exp\!\big((\delta\xi)^\wedge\big),$$
where $\delta\theta\in\mathbb{R}^3$, $\delta\xi\in\mathbb{R}^6$, $(\cdot)^\wedge$ maps vector coordinates to the corresponding Lie algebra element, and $\exp(\cdot)$ denotes the matrix exponential.

For $a=\begin{bmatrix}a_1 & a_2 & a_3\end{bmatrix}^\top\in\mathbb{R}^3$, the $\mathfrak{so}(3)$ wedge is
$$a^\wedge \triangleq
\begin{bmatrix}
0 & -a_3 & a_2\\
a_3 & 0 & -a_1\\
-a_2 & a_1 & 0
\end{bmatrix},$$
so that $a^\wedge b = a\times b$ for all $b\in\mathbb{R}^3$. Furthermore, for $\delta\xi=\begin{bmatrix}\delta\rho\\ \delta\phi\end{bmatrix}\in\mathbb{R}^6$, the $\mathfrak{se}(3)$ wedge is
$$(\delta\xi)^\wedge \triangleq
\begin{bmatrix}
(\delta\phi)^\wedge & \delta\rho\\
0^\top & 0
\end{bmatrix}.$$
Unless stated otherwise, Jacobians are with respect to these right-perturbation coordinates.

Angular velocities follow the same indexing, such that $\vec\omega_{YX}^{\mathcal Y}$ is the angular velocity of frame $\mathcal Y$ relative to frame $\mathcal X$, expressed in frame $\mathcal Y$. With the rotation convention above,
$$\dot R_{XY} = R_{XY}\,\big(\vec\omega_{YX}^{\mathcal Y}\big)^\wedge.$$

We use the following reference frames:
* $\mathcal{I}$: Earth-centered inertial (ECI) frame with origin $O$ at Earth's center of mass.
* $\mathcal{S}$: Inspector spacecraft body-fixed frame with origin $S$ at the inspector's center of mass.
* $\mathcal{G}$: Target body-fixed frame with origin $G$ at a geometric centroid of the initial SLAM-observed point cloud.
* $\mathcal{A}$: Target COM frame with origin $A$ at the target's center of mass. $\mathcal{A}$ is aligned with $\mathcal{G}$ and offset from it by the constant but unknown vector $\vec r_{AG}^{\mathcal G}$.

These frames are mapped to the arena as:
* $\mathcal{I}$: Arena world
* $\mathcal{S}$: ASTROS camera
* $\mathcal{G}$: SLAM-Observed body frame
* $\mathcal{A}$: End Effector/Enforced center of rotation
