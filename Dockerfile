FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# The desktop stays inside the container; only noVNC's HTTP port is published.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgl1-mesa-dri libegl-mesa0 mesa-utils \
      novnc openbox websockify x11-utils x11vnc xterm xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY care_robot/package.xml src/care_robot/package.xml
COPY care_robot_description/package.xml src/care_robot_description/package.xml
COPY care_robot_gazebo/package.xml src/care_robot_gazebo/package.xml
COPY care_robot_bringup/package.xml src/care_robot_bringup/package.xml
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
    && rm -rf /var/lib/apt/lists/*

COPY care_robot src/care_robot
COPY care_robot_description src/care_robot_description
COPY care_robot_gazebo src/care_robot_gazebo
COPY care_robot_bringup src/care_robot_bringup
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --merge-install \
    && colcon test --merge-install --return-code-on-test-failure \
    && colcon test-result --verbose \
    && rm -rf build log src

COPY --chmod=755 docker/entrypoint.sh /usr/local/bin/ros-entrypoint
COPY --chmod=755 docker/start-desktop.sh /usr/local/bin/start-desktop
COPY --chmod=755 docker/check_sim.py /usr/local/bin/check-simulation
RUN useradd --create-home --uid 1001 --shell /bin/bash robot \
    && mkdir -p /data/log \
    && chown -R robot:robot /data \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /home/robot/.bashrc \
    && echo 'source /workspace/install/setup.bash' >> /home/robot/.bashrc

USER robot
WORKDIR /data
ENV DISPLAY=:99 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    ROS_LOG_DIR=/data/log \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
EXPOSE 6080
STOPSIGNAL SIGINT
ENTRYPOINT ["/usr/local/bin/ros-entrypoint"]
CMD ["start-desktop"]
