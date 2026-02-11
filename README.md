
GraspGen2004 — Diffusion-Based Grasp Generation for ROS1
=========================================================

This repository contains the implementation of a diffusion-based grasp generation
pipeline integrated into a ROS1 environment. The system runs inside a Docker container
and provides GPU-accelerated grasp synthesis together with visualization via MeshCat.

---------------------------------------------------------------------
OVERVIEW
---------------------------------------------------------------------

The project consists of:

- Diffusion-based grasp generation
- PyTorch-based model inference
- ROS Noetic integration
- Robokudo message interfaces
- MeshCat visualization
- Docker-based reproducible environment


---------------------------------------------------------------------
REQUIREMENTS
---------------------------------------------------------------------

Host system:

- Ubuntu 20.04
- NVIDIA GPU
- NVIDIA drivers installed
- Docker
- NVIDIA Container Toolkit


Install Docker
--------------

sudo apt install docker.io
sudo usermod -aG docker $USER

IMPORTANT:
Logout and login again after adding the user to the docker group.


Install NVIDIA Container Toolkit
--------------------------------

sudo apt install nvidia-container-toolkit
sudo systemctl restart docker


---------------------------------------------------------------------
INSTALLATION
---------------------------------------------------------------------

1. Clone Repository
-------------------

IMPORTANT: This project uses submodules.

git clone --recurse-submodules <repo-url>
cd GraspGen2004

If already cloned without submodules:

git submodule update --init --recursive


2. Build Docker Image
---------------------

Build the full environment:

./build.sh

This builds:

- PyTorch + CUDA environment
- GraspGen dependencies
- Manifold
- ROS Noetic (ros-base)
- Python dependencies

Build time may take 20–40 minutes.


---------------------------------------------------------------------
BUILDING ROS WORKSPACE
---------------------------------------------------------------------

After starting the container (see below):

cd /GraspGen2004/catkin_ws
catkin build

or alternatively:

catkin_make

Then source the workspace:

source devel/setup.bash

This step only needs to be done once unless ROS packages change.


---------------------------------------------------------------------
RUNNING THE SYSTEM
---------------------------------------------------------------------

The system requires two terminals.


Terminal 1 — Start Docker Container
-----------------------------------

From repository root:

./run.sh

This will:

- Start container with GPU access
- Mount project directory
- Configure ROS networking
- Install python package in editable mode

You are now inside the container shell.


Terminal 2 — Start MeshCat Server
---------------------------------

Open a new terminal and run:

./run_meshcat.sh

This starts the visualization server.


Attach to Running Container
---------------------------

If the container is already running:

./start.sh


---------------------------------------------------------------------
RUNNING THE ROS NODE
---------------------------------------------------------------------

Inside the container:

roscore

In another container terminal:

rosrun <your_package> <your_node>.py

or

roslaunch <your_package> <launchfile>.launch

(depending on your setup)


---------------------------------------------------------------------
MESHCAT VISUALIZATION
---------------------------------------------------------------------

MeshCat runs automatically once the server is started.

Open in browser:

http://localhost:7000


---------------------------------------------------------------------
CONFIGURATIONS TO MAKE IN GRASPING PIPELINE
---------------------------------------------------------------------

(To be filled)

- Parameter configuration
- Model paths
- ROS topic configuration
- Pipeline integration settings


---------------------------------------------------------------------
NOTES ON SUBMODULES
---------------------------------------------------------------------

The repository uses:

catkin_ws/src/robokudo_msgs

as a Git submodule.

After cloning always run:

git submodule update --init --recursive


---------------------------------------------------------------------
TROUBLESHOOTING
---------------------------------------------------------------------

Docker cannot access GPU

Check:

docker run --gpus all nvidia/cuda:12.1.0-base nvidia-smi


ROS topics not visible

Verify:

echo $ROS_MASTER_URI
echo $ROS_IP