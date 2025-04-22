FROM ghcr.io/sloretz/ros:humble-desktop-full AS devcontainer

RUN mkdir -p /ros2_ws/src

ARG ROS_DISTRIBUTION="humble"
ARG FRANKA_ROS2_VERSION="v2.0.2"
ARG FRANKA_DESCRIPTION_VERSION="1.0.1"
ARG LIBFRANKA_VERSION="0.15.0"

# Add non-root user
ARG USERNAME=franka
ENV NON_ROOT_USER=$USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

WORKDIR /home/$USERNAME/ros2_ws/src
RUN /bin/bash -c '\
    git clone --recursive https://github.com/frankarobotics/franka_ros2.git && \
    git clone --recursive https://github.com/frankarobotics/franka_description.git &&\
    git clone --recursive https://github.com/frankarobotics/libfranka.git && \
    (cd libfranka && git checkout ${LIBFRANKA_VERSION} && git submodule update) && \
    (cd franka_description && git checkout ${FRANKA_DESCRIPTION_VERSION} && git submodule update) && \
    (cd franka_ros2 && git checkout ${FRANKA_ROS2_VERSION} && git submodule update) && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/'

WORKDIR /home/$USERNAME/ros2_ws

RUN apt-get update && apt-get install -y wget ros-humble-rmw-cyclonedds-cpp && rosdep install --from-paths src --ignore-src -r -y && apt-get clean && rm -rf /var/lib/apt/lists/*

USER $USERNAME

RUN /bin/bash -c '\
    echo "source /opt/ros/${ROS_DISTRIBUTION}/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRIBUTION}/setup.sh" >> /home/$USERNAME/.profile && \
    echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/ros2_ws/install/setup.sh" >> /home/$USERNAME/.profile'

SHELL ["/bin/bash", "-l", "-c"]

FROM devcontainer AS application

COPY . src/franka_ros2_teleop/

RUN source /opt/ros/${ROS_DISTRIBUTION}/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=off

USER root

ARG TARGETARCH

RUN wget https://github.com/mikefarah/yq/releases/latest/download/yq_linux_${TARGETARCH} -O /usr/local/bin/yq &&\
    chmod +x /usr/local/bin/yq

USER $USERNAME

ENTRYPOINT ["/bin/bash", "-l", "-c", "yq eval '.x-teleop-config' /config.yml > /tmp/franka_ros2_teleop_config.yaml && ros2 launch franka_ros2_teleop teleop.launch.py robot_config_file:=/tmp/franka_ros2_teleop_config.yaml \"$@\""]


