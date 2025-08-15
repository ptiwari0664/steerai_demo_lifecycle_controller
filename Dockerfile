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
COPY code/ ./code

# Build
WORKDIR /ws
RUN . /opt/ros/humble/setup.bash && colcon build --symlink-install

# Runtime env
# ENV QT_QPA_PLATFORM=offscreen (offscreen for headless)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Default entrypoint: launch with source
CMD [ "bash", "-lc", "source /opt/ros/humble/setup.bash && source install/local_setup.bash && exec bash" ]

