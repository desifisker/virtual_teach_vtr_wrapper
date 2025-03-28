ARG UBUNTU_VERSION=20.04
ARG NVIDIA_CUDA_VERSION=11.8.0

FROM nvidia/cuda:${NVIDIA_CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION} 

ARG GROUPID=0
ARG USERID=0
ARG USERNAME=root
ARG HOMEDIR=/root

# Set non-interactive mode.
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

# Install basic dependencies and tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-empy \
    git \
    wget \
    cmake \
    ninja-build \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-system-dev \
    libeigen3-dev \
    libflann-dev \
    libfreeimage-dev \
    libmetis-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libsqlite3-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev \
    libceres-dev \
    nano \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*
    
# Add the ROS Noetic repository (Focal) with its key
RUN sh -c 'echo "deb [signed-by=/usr/share/keyrings/ros-noetic-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-noetic.list' && \
    curl -sSL http://packages.ros.org/ros.key -o /usr/share/keyrings/ros-noetic-archive-keyring.gpg

# Install ROS Noetic and Gazebo packages
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    ros-noetic-robot-localization \
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy \
    ros-noetic-interactive-marker-twist-server \
    ros-noetic-twist-mux \
    python3-netifaces \
    x11-apps \
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    libxi-dev \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep.
RUN rosdep init && rosdep update

# Install Blender 4.3.2.
RUN wget https://download.blender.org/release/Blender4.3/blender-4.3.2-linux-x64.tar.xz -O /tmp/blender.tar.xz && \
    tar -xf /tmp/blender.tar.xz -C /opt && \
    ln -s /opt/blender-4.3.2-linux-x64/blender /usr/local/bin/blender && \
    rm /tmp/blender.tar.xz
    
# Build and install COLMAP.
RUN mkdir -p /colmap
WORKDIR /colmap/src
RUN git clone https://github.com/colmap/colmap.git && \
    cd colmap && \
    git checkout "3.8" && \
    mkdir build && \
    cd build && \
    mkdir -p /build && \
    cmake .. -GNinja "-DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES}" \
        -DCMAKE_INSTALL_PREFIX=/build/colmap && \
    ninja install -j1 
ENV PATH="/build/colmap/bin:${PATH}"

    # Install Miniconda.
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/miniconda && \
    rm /tmp/miniconda.sh
    
# Update PATH to include conda.
ENV PATH="${PATH}:/opt/miniconda/bin"
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}

# Install torch and torchvision for CUDA 11.8
RUN pip install networkx==3.1
#RUN pip install --upgrade pip && pip install torch==2.1.2+cu118 torchvision==0.16.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118
  
# Install CUDA toolkit via conda (ensure conda is installed in the container)
#RUN conda install -y -c "nvidia/label/cuda-11.8.0" cuda-toolkit
  
RUN pip install --upgrade pip
RUN pip install --upgrade numpy

ENV TCNN_CUDA_ARCHITECTURES=75,86

# Install ninja and tiny-cuda-nn (for torch bindings)
#RUN pip install ninja "git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch"

# Create and set up a catkin workspace
#RUN mkdir -p /catkin_ws/src
#WORKDIR /catkin_ws/src
RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src && \
    git clone https://github.com/warthog-cpr/warthog_simulator.git && \
    git clone https://github.com/warthog-cpr/warthog.git && \
    cd /catkin_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make" #&& catkin_make install -DCMAKE_INSTALL_PREFIX=/catkin_ws/install"

RUN wget -O /catkin_ws/src/warthog/warthog_description/urdf/warthog.urdf.xacro \
    https://raw.githubusercontent.com/desifisker/virtual_teach_vtr_wrapper/refs/heads/main/configs/warthog.urdf.xacro
    
RUN wget -O /catkin_ws/src/warthog/warthog_description/meshes/RULER.dae \
    https://raw.githubusercontent.com/desifisker/virtual_teach_vtr_wrapper/refs/heads/main/configs/RULER.dae

RUN wget -O /opt/ros/noetic/share/teleop_twist_joy/config/logitech.config.yaml \
    https://raw.githubusercontent.com/desifisker/virtual_teach_vtr_wrapper/refs/heads/main/configs/logitech.config.yaml
    
RUN wget -O /opt/ros/noetic/share/teleop_twist_joy/launch/teleop.launch \
    https://raw.githubusercontent.com/desifisker/virtual_teach_vtr_wrapper/refs/heads/main/configs/teleop.launch

WORKDIR ${VTRROOT}

# Add ROS Noetic environment setup and alias to the user's bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> $HOME/.bashrc && \
    echo "alias ros1='source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash'" >> $HOME/.bashrc
    
# Copy the entrypoint script into the image.
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Use the entrypoint script, then bash as default command.
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash", "-l"]
