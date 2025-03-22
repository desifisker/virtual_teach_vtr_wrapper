FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu20.04

CMD ["/bin/bash"]

ARG GROUPID=0
ARG USERID=0
ARG USERNAME=root
ARG HOMEDIR=/root
ARG CUDA_ARCH="8.9"

RUN if [ ${GROUPID} -ne 0 ]; then addgroup --gid ${GROUPID} ${USERNAME}; fi \
  && if [ ${USERID} -ne 0 ]; then adduser --disabled-password --gecos '' --uid ${USERID} --gid ${GROUPID} ${USERNAME}; fi

# Default number of threads for make build
ARG NUMPROC=1 #12

ENV DEBIAN_FRONTEND=noninteractive

## Switch to specified user to create directories
USER ${USERID}:${GROUPID}

ENV VTRROOT=${HOMEDIR}/ASRL/vtr3
ENV VTRSRC=${VTRROOT}/src \
  VTRDATA=${VTRROOT}/data \
  VTRTEMP=${VTRROOT}/temp \
  VTRMODELS=${VTRROOT}/models \
  GRIZZLY=${VTRROOT}/grizzly \
  WARTHOG=${VTRROOT}/warthog \
  VTRUI=${VTRSRC}/main/src/vtr_gui/vtr_gui/vtr-gui
  
## Switch to root to install dependencies
USER 0:0

RUN apt-get update && apt-get install -y tzdata && \
    ln -fs /usr/share/zoneinfo/Europe/Berlin /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Install basic packages
RUN apt-get update && apt-get install -y \
    build-essential cmake curl git wget nano lsb-release gnupg2 ca-certificates

# Install X11 libraries and test apps
RUN apt-get update && apt-get install -y \
    x11-apps \
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    libxi-dev \
    && rm -rf /var/lib/apt/lists/*
    
# Add the ROS Noetic repository (Focal) with its key
RUN sh -c 'echo "deb [signed-by=/usr/share/keyrings/ros-noetic-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-noetic.list' && \
    curl -sSL http://packages.ros.org/ros.key -o /usr/share/keyrings/ros-noetic-archive-keyring.gpg

# Install ROS Noetic and Gazebo packages
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    socat \
    gazebo11 libgazebo11-dev \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    ros-noetic-robot-localization \
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy \
    ros-noetic-interactive-marker-twist-server \
    ros-noetic-twist-mux

RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src && \
    git clone https://github.com/warthog-cpr/warthog_simulator.git && \
    git clone https://github.com/warthog-cpr/warthog.git && \
    cd /catkin_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Add ROS Noetic environment setup and alias to the user's bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> $HOME/.bashrc && \
    echo "alias ros1='source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash'" >> $HOME/.bashrc

CMD ["tail", "-f", "/dev/null"]
