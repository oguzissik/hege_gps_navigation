FROM ros:humble-ros-base-jammy

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        astyle \
        build-essential \
        ca-certificates \
        ccache \
        cmake \
        cppcheck \
        file \
        g++ \
        gcc \
        gdb \
        gnupg \
        lcov \
        libfuse2 \
        libxml2-dev \
        libxml2-utils \
        lsb-release \
        make \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-setuptools \
        python3-wheel \
        rsync \
        shellcheck \
        unzip \
        zip \
        bash-completion \
        can-utils \
        curl \
        git \
        iproute2 \
        nano \
        net-tools \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        sudo \
        usbutils \
        ros-humble-desktop \
        ros-humble-diagnostic-updater \
        ros-humble-foxglove-bridge \
        ros-humble-geodesy \
        ros-humble-geographic-msgs \
        ros-humble-imu-tools \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-plotjuggler-ros \
        ros-humble-robot-localization \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-tf-transformations \
        ros-humble-tf2-tools \
        ros-humble-twist-mux \
        ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic: PX4 v1.16 simulator used for the Ackermann rover.
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        gz-harmonic \
        libgz-sim8-dev \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Standalone Agent version recommended for PX4 v1.16 (client uses XRCE v2).
# Build before sourcing ROS so its bundled DDS dependencies are used.
RUN git clone --depth 1 --branch v2.4.3 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/hege-xrce-agent \
    && cmake -S /tmp/hege-xrce-agent -B /tmp/hege-xrce-agent/build \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/hege-xrce-agent/build -j2 \
    && cmake --install /tmp/hege-xrce-agent/build \
    && ldconfig \
    && test -x /usr/local/bin/MicroXRCEAgent \
    && rm -rf /tmp/hege-xrce-agent

# Python modules required by the PX4 v1.16.1 build system.
RUN git clone --depth 1 --branch v1.16.1 \
        https://github.com/PX4/PX4-Autopilot.git /tmp/px4-setup \
    && python3 -m pip install --no-cache-dir \
        -r /tmp/px4-setup/Tools/setup/requirements.txt \
    && rm -rf /tmp/px4-setup

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

ENV DEBIAN_FRONTEND=dialog
ENV ROS_DISTRO=humble
ENV HEGE_REPO=/workspaces/hege_gps_navigation
ENV HEGE_WS=/workspaces/hege_gps_navigation/hege_ws

USER ${USERNAME}
WORKDIR ${HEGE_REPO}

CMD ["bash"]
