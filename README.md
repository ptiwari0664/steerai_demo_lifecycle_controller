# steerai_demo_lifecycle_controller

A **ROS 2 LifecycleNode–based demo controller** for `turtlesim` that demonstrates:

- ✅ **Full LifecycleNode support** (`configure`, `activate`, `deactivate`, `cleanup`, `shutdown`)
- ✅ **Internal operational state machine** (modes: `IDLE`, `CIRCLE`, `ERROR`)
- ✅ **Separation of concerns** — Core algorithm logic is ROS-agnostic and unit-tested
- ✅ **Dynamic parameters** with validation and descriptor metadata
- ✅ **All major ROS 2 communication patterns**:
  - Lifecycle Publisher (`/turtle1/cmd_vel`)
  - Subscriber (`/turtle1/pose`)
  - Services (`/set_mode`, `/toggle_circle`, `/go_home`, `/health`)
  - Client (`/turtle1/teleport_absolute`)
  - Action Server (`/demo_action` implementing `example_interfaces/action/Fibonacci`)
  - Timers

---

## Installation

Install dependencies:

```bash
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-turtlesim
sudo apt install ros-humble-example-interfaces
```

Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repo_url>
cd ..
```

Build:

```bash
colcon build --packages-select steerai_demo_lifecycle_controller
source install/setup.bash
```
## Docker Container Build and Usage
Build the image using Dockerfile from the `steerai_demo_lifecycle_controller` folder

```bash
cd <steerai_demo_lifecycle_controller folder path>
DOCKER_BUILDKIT=1 docker build -t steerai/lifecycle:humble .
```
Run the Container after successful image build

```bash
docker run --rm -it --name steerai_demo --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --env="XAUTHORITY=$XAUTH" --volume="$XAUTH:$XAUTH" --net=host steerai/lifecycle:humble
```

Once inside the container, run below command to lunch the node and leave this node running in terminal

```bash
ros2 launch steerai_demo_lifecycle_controller demo.launch.py
```

Open another terminal:
```bash
docker exec -it steerai_demo bash
# Now you can run:
# Configure and activate lifecycle node
ros2 lifecycle set /turtle_lifecycle_controller configure
ros2 lifecycle set /turtle_lifecycle_controller activate

# Switch to CIRCLE mode
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"

# Enable circle motion and You should now see the turtle moving in circles.
ros2 service call /toggle_circle std_srvs/srv/SetBool "{data: true}"

# While the turtle is moving. You’ll see feedback streaming and a succeeded result while the turtle keeps circling.
ros2 action send_goal /demo_action example_interfaces/action/Fibonacci "{order: 10}"
```


## Launch

Run the turtlesim and lifecycle controller:

```bash
ros2 launch steerai_demo_lifecycle_controller demo.launch.py
```

In a new terminal, activate the node:

```bash
# Configure and activate lifecycle node
ros2 lifecycle set /turtle_lifecycle_controller configure
ros2 lifecycle set /turtle_lifecycle_controller activate
```

## 🎮 Controlling the Turtle

```bash
# Switch to CIRCLE mode
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"

# Enable circle motion
ros2 service call /toggle_circle std_srvs/srv/SetBool "{data: true}"

```
You should now see the turtle moving in circles.


## 🛠 Adjust Parameters

```bash
ros2 param set /turtle_lifecycle_controller linear_speed 0.8
ros2 param set /turtle_lifecycle_controller angular_speed 1.5
```

## Action Example
While the turtle is moving. You’ll see feedback streaming and a succeeded result while the turtle keeps circling.
```bash
ros2 action send_goal /demo_action example_interfaces/action/Fibonacci "{order: 10}"
```

## 📊 Flow Diagram

```mermaid
flowchart TD
    A[Launch Node] --> B[UNCONFIGURED]
    B -->|on_configure| C[INACTIVE]
    C -->|on_activate| D[ACTIVE]
    D -->|Service: set_mode/toggle_circle| E[Circle Motion]
    D -->|Service: go_home| F[Teleport to Center]
    D -->|Action: Fibonacci| G[Compute Sequence]
    D -->|on_deactivate| C
    C -->|on_cleanup| B
    B -->|on_shutdown| H[FINALIZED]
```


















# installation
```
apt install python3-colcon-common-extensions
apt install ros-humble-turtlesim
apt install ros-humble-example-interfaces

```



## To create the package 

'ros2 pkg create steerai_demo_lifecycle_controller --build-type ament_python --dependencies rclpy lifecycle_msgs std_srvs geometry_msgs turtlesim'


## Test
`colcon test --packages-select steerai_demo_lifecycle_controller`
