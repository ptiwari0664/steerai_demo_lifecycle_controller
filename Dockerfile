# Dockerfile
FROM ros:humble-ros-base
SHELL ["/bin/bash", "-lc"]

# System deps
RUN apt-get update && apt-get install -y \
    xvfb \
    ros-humble-turtlesim \
    ros-humble-example-interfaces \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Workspace
WORKDIR /ws/src
# Copy only the package first for better layer caching
COPY steerai_demo_lifecycle_controller ./steerai_demo_lifecycle_controller
COPY launch ./steerai_demo_lifecycle_controller/launch
COPY package.xml ./steerai_demo_lifecycle_controller/
COPY setup.py ./steerai_demo_lifecycle_controller/
COPY resource ./steerai_demo_lifecycle_controller/resource
COPY test ./steerai_demo_lifecycle_controller/tests

# Build
WORKDIR /ws
RUN . /opt/ros/humble/setup.bash && colcon build --symlink-install

# Runtime env
ENV QT_QPA_PLATFORM=offscreen
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Default entrypoint: headless launch
# (xvfb avoids Qt display issues inside containers)
CMD [ "bash", "-lc", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && exec bash" ]
# CMD xvfb-run -s "-screen 0 1024x768x24" bash -lc "\
#   . /opt/ros/humble/setup.bash && \
#   . /ws/install/setup.bash && \
#   ros2 launch steerai_demo_lifecycle_controller demo.launch.py"
