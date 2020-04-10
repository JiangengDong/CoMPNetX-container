# Docker Image for Openrave

This repository provides a docker image for OpenRAVE, OMPL, ROS Kinetic and some necessary ROS packages for the Baxter robot. Some OpenRAVE models from the [CoMPS](https://sourceforge.net/projects/comps/) are also included in this image, under the `/home/atlas/ormodels` folder. The username and password for this image are both "**atlas**".

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

* GUI using the host's X-server

Some other libraries are also installed as dependencies. We do not list them here because they are not intended.

## quick start

### Get the docker image

There are three ways to get an image. 

1. You can pull it from the [docker hub](https://hub.docker.com/repository/docker/jiangengdong/openrave). Note that we did not optimize the image size, so the total size is 9.75GB.

1. If your network is not fast enough, you may want to [build it from Dockerfile](#build-the-docker-image-from-dockerfile). It takes 24 hours to build this image on a laptop with Intel® Core™ i5-4210H and GeForce GTX 960M, so please think twice before you start the building.

1. Otherwise, you can [copy a image](#copy-the-docker-image-from-your-colleague) from your colleague's computer.

#### Build the docker image from Dockerfile

It will take hours to build this image, so Please make Sure you satisfy all the [requirements](#requirement). Put the [catkin_ws](https://drive.google.com/open?id=1XXzqqz3OuNFN-ZFKV-SLdEO23JVqeB4M) folder, the [ormodel](https://drive.google.com/open?id=1w_S6udx6ELKEkD_SLu1-z4dZqjNiD0FJ) folder and the [Dockerfile](./Dockerfile) under the same folder and execute the following command to build the image. 

```bash
docker build --rm -f Dockerfile -t jiangengdong/openrave .
```

Do not change the image name unless you know what you are doing, because we use the name "**jiangengdong/openrave**" in the [.devcontainer/devcontainer.json](./.devcontainer/devcontainer.json), which is later used in the [development step](#use-vscode-to-develop-inside-the-container).

#### Copy the docker image from your colleague

On your colleague's computer, save the image to a `*.tar.gz` file.

```bash
docker save -o <path for generated tar file> jiangengdong/openrave
```

Copy the generated `*.tar.gz` file to your own computer and load it.

```bash
docker load -i <path to image tar file>
```

### Enable container GUI

The container displays GUI by reusing the X11 unix socket of the host. So we need to give the container the access to the X11 server.

```bash
xhost +
```

### Start a bash inside the container

The [start_container.sh](./start_container.sh) provides a simple way of opening a bash in the container. You can use it to test if this container works properly.

```bash
chmod u+x start_container.sh
./start_container.sh
```

### Use VSCode to develop inside the container

We recommand you use the [VSCode](https://code.visualstudio.com/) and its [Remote-Containers](https://code.visualstudio.com/docs/remote/containers) extension to develop in the container. We provide a [.devcontainer/devcontainer.json](./.devcontainer/devcontainer.json) for you to start quickly.

1. Install [VSCode](https://code.visualstudio.com/) and its [Remote-Containers](https://code.visualstudio.com/docs/remote/containers) extension.

1. Copy the [.devcontainer](./.devcontainer) folder to your work folder, say `/home/peter/src`.

1. Open your work folder in VSCode using "**File-Open Folder**".

1. A notification will pop up in the right bottom corner asking you to reopen this folder in the container. Click "**Reopen in Container**". 

1. Now you can develop in the container.

**Warning**: All the data in the container will be destroyed upon the container's closure, except those in your work folder. Do not store anything outside this folder if you want to use it later.

## Further reference

1. [Docker's Guide](https://docs.docker.com/)

1. [VSCode's Remote Development Guide for Containers](https://code.visualstudio.com/docs/remote/containers)

1. [Installing OpenRAVE on Ubuntu 16.04](https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html)

1. [Constrained Manipulation Planning Suite](https://sourceforge.net/projects/comps/)

1. [The Open Motion Planning Library](https://ompl.kavrakilab.org/)

1. [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)