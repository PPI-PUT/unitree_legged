FROM osrf/ros:humble-desktop-full

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    curl \
    python3 \
    python3-dev \
    python3-vcstool \ 
    ca-certificates \
    net-tools \
    iputils-ping \ 
    nano \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt remove -y default-jdk
RUN apt autoremove -y

WORKDIR /root
RUN curl -L -o lcm-v1.4.0.tar.gz https://github.com/lcm-proj/lcm/archive/refs/tags/v1.4.0.tar.gz
RUN tar -xzvf lcm-v1.4.0.tar.gz \
    && rm lcm-v1.4.0.tar.gz \
    && cd lcm-1.4.0 \
    && mkdir build \
    && cd build \
    && cmake .. \ 
    && make install

WORKDIR /root/ros2_ws/src

RUN git clone https://github.com/PPI-PUT/unitree_legged.git

WORKDIR /root/ros2_ws/src/unitree_legged
RUN mkdir -p a1_description/test \
    && vcs import < deps.repos \ 
    && cd unitree_a1_legged/unitree_legged_sdk \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make


ENV UNITREE_SDK_VERSION=3_3
ENV UNITREE_LEGGED_SDK_PATH=/root/ros2_ws/src/unitree_legged/unitree_a1_legged/unitree_legged_sdk

WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On -Wall -Wextra -Wpedantic"

CMD ["bash"]
