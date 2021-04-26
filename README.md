# Docker Image for CoMPNet

**Note**: This branch is no longer maintained because the vnc version is too complex to use. We do not guarantee the stability of this branch in the future. 

This repository provides a docker image for the [CoMPNet] project, which sets up OpenRAVE, OMPL, ROS Kinetic and some necessary ROS packages for the Baxter robot. The username and password for this image are both "**atlas**".

## Requirement

### Common

1. NVIDIA GPU and Intel CPU

1. ubuntu 16.04

1. x11-xserver-utils

1. docker 19.03.7 (which supports `--gpus` argument)

1. [NVIDIA Driver](https://github.com/NVIDIA/nvidia-docker/wiki/Frequently-Asked-Questions#how-do-i-install-the-nvidia-driver)

1. [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker)

### Build only

1. the [catkin_ws](https://drive.google.com/open?id=1XXzqqz3OuNFN-ZFKV-SLdEO23JVqeB4M) folder 

1. the [ormodel](https://drive.google.com/open?id=1w_S6udx6ELKEkD_SLu1-z4dZqjNiD0FJ) folder

## Environment and Softwares

The docker image provides the following libraries and environments:

* Ubuntu16.04

* OpenRAVE 0.9

* OMPL 1.4.2

* ROS Kinetic

* CUDA 10.0

* CuDNN 7.6.5.32

* PyTorch 1.4.2

* GUI

Some other libraries are also installed as dependencies. We do not list them here because they are not intended.

## Quick start

```bash 
wget https://raw.githubusercontent.com/JiangengDong/OpenRAVE-Docker/master/start_container.sh
chmod u+x ./start_container.sh
xhost + && ./start_container.sh cudagl
OR
./start_container.sh cudavnc
```

We provide two different ways of visualization. 

- `cudagl`: The first method depends on X11 forwarding. When you have an X server on your host, you can use this method to relay the signals from inside the container to the host. For more detail, please read [cudagl/README](cudagl/README.md).

- `cudavnc`: The second method depends on VNC. If you are using the image on a server, or if you do not have an X server locally, you will have to use VNC. For more detail, please read [cudavnc/README](cudavnc/README.md).

We cannot provide the two visualization in the same image yet. Sorry for your inconvenience.


## TODO

- Rebuild torch_base with pytorch v1.4.1.

- Try to use multi-stage build