ARG UBUNTU_VERSION=20.04
ARG NVIDIA_CUDA_VERSION=11.8.0

FROM nvidia/cuda:${NVIDIA_CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION} 

ARG GROUPID=0
ARG USERID=0
ARG USERNAME=root
ARG HOMEDIR=/root
ARG CUDA_ARCHITECTURES="90;89;86;80;75;70;61"

RUN if [ ${GROUPID} -ne 0 ]; then addgroup --gid ${GROUPID} ${USERNAME}; fi \
  && if [ ${USERID} -ne 0 ]; then adduser --disabled-password --gecos '' --uid ${USERID} --gid ${GROUPID} ${USERNAME}; fi

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
  NERF=${VTRROOT}/virtual_teach_vtr_wrapper/src/nerfstudio/nerfstudio \
  CATKIN=${VTRROOT}/virtual_teach_vtr_wrapper/catkin_ws \
  VTRUI=${VTRSRC}/main/src/vtr_gui/vtr_gui/vtr-gui

RUN echo "alias build_ui='npm --prefix ${VTRUI} install ${VTRUI}; npm --prefix ${VTRUI} run build'" >> ~/.bashrc
RUN echo "alias build_vtr='source /opt/ros/humble/setup.bash; cd ${VTRSRC}/main; colcon build --symlink-install'" >> ~/.bashrc

## Switch to root to install dependencies
USER 0:0

# Update PATH.
ENV PATH="/build/colmap/bin:${PATH}"
ENV PATH="${PATH}:/opt/miniconda/bin"
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV TCNN_CUDA_ARCHITECTURES=75,86
ENV NERFSTUDIO_DIR=/nerfstudio

# Install basic dependencies and tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    python3.8 \
    python3.8-dev \
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
    libgl1 \
    libgoogle-glog0v5 \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    libcgal-dev \
    libceres-dev \
    libc6 \
    libfreeimage3 \
    libgcc-s1 \
    nano \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*
    
# ADD ROS AND ROS PACKAGES #######################################################################################################################################################
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
    python3-rosdep \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep.
RUN rosdep init && rosdep update

# BLENDER 4.3.2. ##################################################################################################################################################################
RUN wget https://download.blender.org/release/Blender4.3/blender-4.3.2-linux-x64.tar.xz -O /tmp/blender.tar.xz && \
    tar -xf /tmp/blender.tar.xz -C /opt && \
    ln -s /opt/blender-4.3.2-linux-x64/blender /usr/local/bin/blender && \
    rm /tmp/blender.tar.xz
    
# COLMAP 3.10 #####################################################################################################################################################################
RUN mkdir -p /colmap
WORKDIR /colmap/src
RUN git clone https://github.com/colmap/colmap.git && \
    cd colmap && \
    git checkout "3.10" && \
    mkdir build && \
    cd build && \
    mkdir -p /build && \
    cmake .. -GNinja "-DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES}" \
        -DCMAKE_INSTALL_PREFIX=/build/colmap && \
    ninja install -j1 
    
#WORKDIR ${VTRROOT}

# MINICONDA #####################################################################################################################################################################
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/miniconda && \
    rm /tmp/miniconda.sh
    
# NERFSTUDIO #####################################################################################################################################################################
# Create the nerfstudio conda environment and perform pip installations inside it.
RUN bash -c "source /opt/miniconda/etc/profile.d/conda.sh && \
    conda create --name nerfstudio -y python=3.8 && \
    conda activate nerfstudio && \
    pip install networkx==3.1 && \
    pip install 'fsspec<2023.0.0' && \
    pip install --no-cache-dir --upgrade pip 'setuptools<70.0.0' && \
    pip install --no-cache-dir torch==2.1.2+cu118 torchvision==0.16.2+cu118 'numpy<2.0.0' --extra-index-url https://download.pytorch.org/whl/cu118 && \
    git clone --branch master --recursive https://github.com/cvg/Hierarchical-Localization.git /opt/hloc && \
    cd /opt/hloc && git checkout v1.4 && python3.8 -m pip install --no-cache-dir . && \
    cd ~ && \
    TCNN_CUDA_ARCHITECTURES=${TCNN_CUDA_ARCHITECTURES} pip install --no-cache-dir 'git+https://github.com/NVlabs/tiny-cuda-nn.git@b3473c81396fe927293bdfd5a6be32df8769927c#subdirectory=bindings/torch' && \
    pip install --no-cache-dir pycolmap==0.6.1 pyceres==2.1 omegaconf==2.3.0 && \
    conda deactivate"

# Clone nerfstudio into a backup folder
RUN git clone --branch main https://github.com/nerfstudio-project/nerfstudio.git /tmp/nerfstudio_default

# WARTHOG GAZEBO SIM CUSTOM CHANGES ##############################################################################################################################################
RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src && \
    git clone https://github.com/warthog-cpr/warthog_simulator.git && \
    git clone https://github.com/warthog-cpr/warthog.git && \
    cd /catkin_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make && catkin_make install -DCMAKE_INSTALL_PREFIX=/catkin_ws/install"

RUN wget -O /catkin_ws/src/warthog/warthog_description/urdf/warthog.urdf.xacro \
    "https://raw.githubusercontent.com/desifisker/virtual_teach_vtr_wrapper/refs/heads/main/configs/warthog.urdf.xacro"

RUN wget -O /catkin_ws/src/warthog/warthog_description/meshes/RULER.dae \
    "https://raw.githubusercontent.com/desifisker/virtual_teach_vtr_wrapper/refs/heads/main/configs/RULER.dae"

RUN wget -O /opt/ros/noetic/share/teleop_twist_joy/config/logitech.config.yaml \
    "https://raw.githubusercontent.com/desifisker/virtual_teach_vtr_wrapper/refs/heads/main/configs/logitech.config.yaml"
    
RUN wget -O /opt/ros/noetic/share/teleop_twist_joy/launch/teleop.launch \
    "https://raw.githubusercontent.com/desifisker/virtual_teach_vtr_wrapper/refs/heads/main/configs/teleop.launch"

# Save a backup copy of the folder for seeding the persistent volume later
RUN cp -r /catkin_ws /catkin_ws_default

# Add extra dependencies Scripts needs
RUN pip3 install numpy scipy pandas matplotlib opencv-python open3d pylgmath pycollada

WORKDIR ${VTRROOT}

# Add ROS Noetic environment setup and alias to the user's bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> $HOME/.bashrc && \
    echo ". /opt/miniconda/etc/profile.d/conda.sh" >> ~/.bashrc && \
    echo "alias ros1='source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash'" >> $HOME/.bashrc
    
# Change ownership of backup directories so that the non-root user can access them
RUN chown -R ${USERID}:${GROUPID} /tmp/nerfstudio_default /catkin_ws_default

# Copy the entrypoint script into the image.
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERID}:${GROUPID}

# Use the entrypoint script, then bash as default command.
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash", "-l"]
