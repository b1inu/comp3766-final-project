# comp3766-final-project

Project that implements Analytical and Numerical Inverse Kinematics for a 6R PUMA robot.

## Dependencies

- ROS2 Jazzy
- Python 3.10+
- `modern_robotics` (included in Docker)
- `transform3d` (included in Docker)
  - to install manually:
    - `pip install modern_robotics transforms3d`

---

## How to Run

### Option 1 - WSL/Linux

#### 1. Clone the repository

```bash
cd ~/comp3766-final-project  # or wherever you cloned it
```

#### 2. Source ROS2 and build the workspace

```bash
source /opt/ros/jazzy/setup.bash
colcon build
. install/setup.bash
```

#### 3. Run the nodes

```bash
# Numerical Inverse Kinematics
ros2 run inverse_kinematics_pkg numerical_inverse_kinematics
# Analytical Inverse Kinematics
ros2 run inverse_kinematics_pkg analytical_inverse_kinematics
# Goal Pose Publisher for Numerical and Analytical Inverse Kinematics
ros2 run inverse_kinematics_pkg goal_pose_node
```

### Option 2 - Docker

#### 1. Clone the repository and build the image

```bash
git clone https://github.com/b1inu/comp3766-final-project.git
cd comp3766-final-project
docker build -t ik_project .
```

#### 2. Run the container

```bash
docker run -it --rm ik_project
```

#### 3. Run the nodes inside the container

```bash
source /opt/ros/jazzy/setup.bash
ros2 run inverse_kinematics_pkg numerical_inverse_kinematics
ros2 run inverse_kinematics_pkg analytical_inverse_kinematics
ros2 run inverse_kinematics_pkg goal_pose_node
```
