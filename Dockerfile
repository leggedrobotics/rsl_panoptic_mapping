# Start from the NVIDIA CUDA base image with cuDNN 8
FROM nvidia/cuda:12.2.2-cudnn8-devel-ubuntu22.04 

ARG DRIVER=535

# Set environment variables
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,video,compute,utility
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    PATH=/usr/local/cuda/bin:$PATH \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# Install graphics drivers
RUN apt update && apt install -y libnvidia-gl-${DRIVER} \
    && rm -rf /var/lib/apt/lists/*

# Needed for string substitution
SHELL ["/bin/bash", "-c"]
ENV TERM=xterm-256color

# Install necessary packages for Python and OpenCV dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-dev \
        git \
        wget \
        curl \
        ca-certificates \
        build-essential \
        libqt5core5a \
        libqt5gui5 \
        libqt5widgets5 \
        qtbase5-dev \
        libgl1 \
        libglib2.0-0 && \
    rm -rf /var/lib/apt/lists/*

# Specify which ROS 2 distro you want to use
ARG ROS_DISTRO=humble

# Set up sources.list and keys for ROS 2
RUN apt update && apt install -y locales && locale-gen en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    apt update && apt install -y software-properties-common && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
         | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
          http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
         | tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null

# Setup environment variables for ROS 2
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_PYTHON_VERSION=3 \
    COLCON_HOME=/opt/colcon \
    ROS_ROOT=/opt/ros/${ROS_DISTRO}

ENV PATH="/opt/ros/${ROS_DISTRO}/bin:${PATH}"
ENV LD_LIBRARY_PATH="/opt/ros/${ROS_DISTRO}/lib:${LD_LIBRARY_PATH}"
ENV PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:${PYTHONPATH}"

# Upgrade pip to the latest version
RUN python3 -m pip install --upgrade pip

# Install PyTorch (compatible with CUDA 11.8) + other Python packages
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
RUN pip3 install matplotlib opencv-python imageio

# Install ROS 2 desktop-full and other ROS tools
RUN apt update && apt install -y \
      ros-${ROS_DISTRO}-desktop-full \
      python3-rosdep \
      python3-colcon-common-extensions \
      python3-vcstool \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init && rosdep update

# Source ROS 2 setup script in .bashrc for interactive shells
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Install Navigation2 stack
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-octomap-msgs \ 
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------
# Create ROS 2 workspace and copy package
# ---------------------------------------

# 1) Make a workspace called ros2_ws with a src/ folder
WORKDIR /workspace/ros2_ws
RUN mkdir -p src

# 2) Copy your dynamic_mapping package into src
#    (Assuming the Docker build context has a local folder named dynamic_mapping)
#COPY ./dynamic_mapping src/dynamic_mapping

# 3) Build the workspace
#RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
#    colcon build --symlink-install
#
## 4) Source the newly built workspace on container start (also in .bashrc)
#RUN echo "source /workspace/ros2_ws/install/setup.bash" >> ~/.bashrc

# ---------------------------------------
# Set the default command to bash
# ---------------------------------------
CMD ["bash"]
