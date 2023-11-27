FROM osrf/ros:noetic-desktop-full-focal

RUN useradd ros1

# add password to ros1 user
RUN echo "ros1:ros" | sudo chpasswd

RUN usermod -a -G sudo ros1

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-rosdep \
    python3-pip \
    python3-rosinstall \
    python3-vcstools \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# install aditional dependencies
RUN apt-get update && apt-get install -y vim \
    ros-noetic-ros-numpy \
    ros-noetic-rviz-visual-tools