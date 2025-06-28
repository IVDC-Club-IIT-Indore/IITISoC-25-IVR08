FROM ubuntu:20.04

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Basic tools
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common

# Add ROS repository key and source
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# Install ROS Noetic
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-pip \
    git \
    vim

# Initialize rosdep
RUN rosdep init && rosdep update

# Install MAVROS and geographiclib datasets (needed for MAVROS to function properly)
RUN apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras \
 && /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# Set up environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Create catkin workspace
WORKDIR /root/catkin_ws
RUN mkdir -p src \
 && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make" \
 && echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Default command
CMD ["/bin/bash"]
