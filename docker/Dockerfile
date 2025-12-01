# Python version: 3.10
# CUDA version: 12.1 (Recommended)
# Software Requirements:
#   - Ray tracing: NVIDIA Driver >= 470
#   - Denoising (OIDN): NVIDIA Driver >= 520
#
# Additional Requirements for Docker:
#   When running this container, ensure the following environment variable is set:
#     -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
#   
#   IMPORTANT: The 'graphics' capability is essential. 
#   Omitting it may result in segmentation faults due to missing Vulkan support.

FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04

# Set required NVIDIA driver capabilities
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

# Set Python version
ENV PYTHON_VERSION=3.10

# Install Vulkan (required for graphics rendering)
RUN apt-get update && apt-get install -y \
    libvulkan1 \
    mesa-vulkan-drivers \
    vulkan-tools \
    && rm -rf /var/lib/apt/lists/*

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    ffmpeg \
    unzip \
    x11-apps \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install Miniforge (conda-forge based, no ToS required)
RUN wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh -O /tmp/miniforge.sh && \
    bash /tmp/miniforge.sh -b -p /opt/conda && \
    rm /tmp/miniforge.sh

ENV PATH=/opt/conda/bin:$PATH

# Initialize conda and create environment
RUN conda init bash && \
    conda config --set always_yes true && \
    conda config --set changeps1 false && \
    conda create -n RoboTwin python=3.10 -y

# Activate conda environment for subsequent commands
SHELL ["conda", "run", "-n", "RoboTwin", "/bin/bash", "-c"]

# Git clone RoboTwin repository
WORKDIR /workspace
RUN git clone https://github.com/RoboTwin-Platform/RoboTwin.git
WORKDIR /workspace/RoboTwin

# Install RoboTwin dependencies and CuRobo
RUN bash script/_install.sh || true

# Update CuRobo embodiment config path if needed
RUN python script/update_embodiment_config_path.py || true

# Note: If you are not using 3D data, a failed installation of pytorch3d 
# will not affect the functionality of the project.

# Verify ffmpeg installation
RUN ffmpeg -version

# Clean up the copied files (will be replaced by volume mount at runtime)
RUN rm -rf /workspace/RoboTwin/*

# Set default conda environment
ENV CONDA_DEFAULT_ENV=RoboTwin

WORKDIR /workspace

# Build command:
#   docker build -t robotwin:latest -f docker/Dockerfile .
#
# Run command with X11 support:
#   xhost +local:docker
#   docker run -it --rm \
#     --gpus all \
#     -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
#     -v ${PWD}:/workspace \
#     --name robotwin \
#     robotwin:latest \
#     /bin/bash
#
# After exiting, restore X11 security:
#   xhost -local:docker
