ARG FROM_IMAGE=docker.io/ros:noetic
ARG OVERLAY_WS=/catkin_ws

FROM $FROM_IMAGE

ARG OVERLAY_WS
ENV OVERLAY_WS=$OVERLAY_WS

RUN apt-get update && apt-get install -y \
    clangd \
    git \
    ros-$ROS_DISTRO-catkin-virtualenv \
    python3-testresources \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p $OVERLAY_WS/src/franka_ros
WORKDIR $OVERLAY_WS

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin init --workspace $OVERLAY_WS && \
    catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
    catkin build

RUN sed --in-place --expression \
        '$isource "$OVERLAY_WS/devel/setup.bash"' \
        /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
