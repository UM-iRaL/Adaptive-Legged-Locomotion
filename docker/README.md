<!-- TOC tocDepth:2..3 chapterDepth:2..6 -->

- [1. On host](#1-on-host)
  - [1.1. Install docker](#11-install-docker)
  - [1.2. Enable CUDA in docker](#12-enable-cuda-in-docker)
  - [1.3. get ROS noetic images](#13-get-ros-images)
  - [1.4. run ROS noetic images](#14-run-ros-images)
- [2. Inside docker image](#2-inside-docker-image)
  - [2.1. basic setup](#21-basic-setup)
  - [2.2. CUDA & pytorch setup](#22-cuda-pytorch-setup)


<!-- /TOC -->
# Docker Setup

## 1. On host

### 1.1. Install docker

follow: https://docs.docker.com/engine/install/


### 1.2. Enable CUDA in docker

Follow: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

```bash
# on host, install
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
  && \
    sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# configure
sudo nvidia-ctk runtime configure --runtime=docker
cat /etc/docker/daemon.json # double check
sudo systemctl restart docker
```

### 1.3. get ROS images

```bash
docker pull osrf/ros:noetic-desktop-full
```

### 1.4. run ROS images

```bash
# needed for X11 forward
xhost +local:docker

# start ros container: "sandbox" is the name of dir; "ros_name" is the name of container()
docker run -it \
    --workdir=/sandbox/ \
    -v $HOME:/sandbox/ \
    -e HOME=/root/ \
    -e "QT_X11_NO_MITSHM=1" \
    -e GDK_SCALE \
    -e GDK_DPI_SCALE \
    -e DISPLAY=unix$DISPLAY \
    --env="DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --name=ssi_mpc_legged \
    --privileged \
    --network host \
    -v /etc/timezone:/etc/timezone:ro \
    -v /etc/localtime:/etc/localtime:ro \
    osrf/ros:noetic-desktop-full

# # attach to ros container
# docker attach quad_sdk
```

## 2. Inside docker image


### 2.1. basic setup
```bash
# test X11 forwarding
rviz2


# install some tools, `-E` for proxy envs
sudo -E apt update
sudo -E apt install -y tmux zsh git vim make cmake tree aptitude \
                       htop curl rsync software-properties-common \
                       locales mlocate aria2 python3-pip
```

### 2.2. CUDA & pytorch setup

```bash
# Download
aria2c https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
aria2c https://download.pytorch.org/whl/cu118/torch-2.1.0%2Bcu118-cp310-cp310-linux_x86_64.whl
aria2c https://download.pytorch.org/whl/cu118/torchvision-0.16.0%2Bcu118-cp310-cp310-linux_x86_64.whl

# install CUDA to /usr/local/cuda-11.8
# !!! don't install driver, only select the cuda lib
bash cuda_11.8.0_520.61.05_linux.run

# add the following lines to .bashrc or .zshrc
export CUDA_ROOT=/usr/local/cuda-11.8
export PATH=/usr/local/cuda-11.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH
export CUDA_INSTALL_DIR=/usr/local/cuda-11.8
export LD_LIBRARY_PATH=/opt/cudnn-linux-x86_64-8.9.3.28_cuda11-archive/lib:$LD_LIBRARY_PATH
export CUDNN_INSTALL_DIR=/opt/cudnn-linux-x86_64-8.9.3.28_cuda11-archive
export CUDNN_ROOT_DIR=$CUDNN_INSTALL_DIR
```


### 2.3. Fix GPG error
```bash
rm -f /etc/apt/sources.list.d/ros*.list
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
  gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
  > /etc/apt/sources.list.d/ros-latest.list
sudo apt update
```
