#############################################
# BASE IMAGE: GRASPGEN + PYTORCH
#############################################
FROM nvcr.io/nvidia/pytorch:23.04-py3 AS base
#https://docs.nvidia.com/deeplearning/frameworks/pytorch-release-notes/rel-23-04.html#

ENV DEBIAN_FRONTEND=noninteractive

# Basic system dependencies
RUN apt update && apt install -y \
    tmux \
    libosmesa6-dev \
    git \
    curl \
    lsb-release \
    sudo \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip

#############################################
# GRASPGEN DEPENDENCIES
#############################################

RUN pip install h5py hydra-core matplotlib meshcat scikit-learn scipy tensorboard \
    trimesh==4.5.3

RUN pip install scene-synthesizer[recommend]

RUN pip install torch==2.1.0 torchvision

RUN pip install torch-cluster -f https://data.pyg.org/whl/torch-2.1.0+cu121.html
# RUN pip install imageio pickle5 opencv-python python-fcl
RUN pip install imageio pickle5 python-fcl
RUN apt update && apt install -y python3-opencv

# pyrender + fallback OpenGL
RUN pip install pyrender==0.1.45 pyglet==2.1.6 \
    && pip install PyOpenGL==3.1.5

# pointnet2
COPY pointnet2_ops pointnet2_ops
RUN pip install ./pointnet2_ops

# diffusion
RUN pip install diffusers==0.11.1 timm==1.0.15
RUN pip install huggingface-hub==0.25.2

# PointTransformer V3
RUN pip install addict yapf==0.40.1 tensorboardx sharedarray torch-geometric \
    && pip install torch-scatter -f https://data.pyg.org/whl/torch-2.1.2+cu121.html \
    && pip install spconv-cu120

# For analytic IK and geometry
RUN pip install qpsolvers[clarabel]
RUN pip install yourdfpy==0.0.56

# Repeated packages (kept for safety)
RUN pip install trimesh==4.5.3 timm==1.0.15

# dataset + Manifold + LFS client
RUN pip install webdataset \
    && curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash \
    && apt update && apt install -y git-lfs \
    && rm -rf /var/lib/apt/lists/*

RUN pip install objaverse==0.1.7

RUN mkdir -p /install \
    && git clone --recursive -j8 https://github.com/hjwdzh/Manifold.git /install/Manifold

RUN mkdir -p /install/Manifold/build \
    && cd /install/Manifold/build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc)

ENV PATH="${PATH}:/install/Manifold/build/"

#############################################
# INSTALL ROS NOETIC (minimal = ros-base)
#############################################

# Add ROS repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
       | apt-key add -

# Install ROS core + catkin tools
RUN apt update && apt install -y \
    ros-noetic-ros-base \
    ros-noetic-catkin \
    ros-noetic-vision-msgs \
    ros-noetic-tf \
    ros-noetic-ros-numpy \
    ros-noetic-cv-bridge \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# RUN source /opt/ros/noetic/setup.bash && \
#     cd /GraspGen2004/catkin_ws && \
#     catkin build

# add source files to .bashrc to have them in each terminal already available
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /GraspGen2004/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN echo "export PYTHONPATH=/GraspGen2004/code:\$PYTHONPATH" >> /root/.bashrc

#############################################
# FINAL WORKDIR
#############################################
WORKDIR /GraspGen2004/
