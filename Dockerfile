ARG FROM_IMAGE=docker.io/ros:noetic
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src

RUN apt-get update && apt-get install -y \
    git \
    && rm -rf /var/lib/apt/lists/*

COPY . ./franka_ros

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update \
    && apt-get install -y python3-catkin-tools ros-$ROS_DISTRO-catkin-virtualenv python3-testresources nlohmann-json3-dev \
    && rosdep install -y \
      --from-paths \
        src/franka_ros/franka_teleop \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin init && \
    catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build franka_teleop && \
    rm -rf build log

FROM docker.io/ros:noetic-ros-core AS runner

ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update \
    && apt-get install -y python3-rosdep \
    && rosdep init && rosdep update --rosdistro $ROS_DISTRO && rosdep install -y \
      --from-paths \
        src/franka_ros/franka_teleop \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder $OVERLAY_WS/install ./install

# source entrypoint setup
ENV OVERLAY_WS=$OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

ENV ROS_IP=192.168.42.1
ENV ROS_MASTER_URI=http://192.168.42.1:11311
USER root
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
