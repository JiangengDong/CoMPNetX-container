# jiangengdong/openrave:cudavnc

## How to get the docker image

There are three ways to get an image. 

- You can pull `jiangengdong/openrave:cudavnc` from DockerHub. Note that we did not optimize the image size, so the total size is 9.75GB.

- If your network is not fast enough, you may want to [build it from Dockerfile](#build-the-docker-image-from-dockerfile). It takes 24 hours to build this image on a laptop with Intel® Core™ i5-4210H and GeForce GTX 960M, so please think twice before you start the building.

- Otherwise, you can [copy a image](#copy-the-docker-image-from-your-colleague) from your colleague's computer.

### Build the docker image from Dockerfile

It will take hours to build this image, so please make Sure you satisfy all the [requirements](../README.md#requirement). Put the [catkin_ws](https://drive.google.com/open?id=1XXzqqz3OuNFN-ZFKV-SLdEO23JVqeB4M) folder, the [ormodel](https://drive.google.com/open?id=1w_S6udx6ELKEkD_SLu1-z4dZqjNiD0FJ) folder and the [Dockerfile](./Dockerfile) under the same folder and execute the following command to build the image. 

```bash
docker build --rm -f "Dockerfile" -t jiangengdong/openrave:cudavnc .
```

Do not change the image name unless you know what you are doing, because we use the name "**jiangengdong/openrave**" in the [.devcontainer/devcontainer.json](./.devcontainer/devcontainer.json), which is later used in the [VSCode step](#use-vscode-to-develop-inside-the-container).

### Copy the docker image from your colleague

On your colleague's computer, save the image to a `*.tar.gz` file.

```bash
docker save -o <path for generated tar file> jiangengdong/openrave:cudavnc
```

Copy the generated `*.tar.gz` file to your own computer and load it.

```bash
docker load -i <path to image tar file>
```

## How to use GUI

This image implements visualization via VNC. We run an `Xvfb` server and an `X11vnc` server inside the container, so everything can be accessed from port 5900. Remember to publish this port, otherwise you cannot connect to the container. You can take a look at [start_container.sh](../start_container.sh) to see how to write the options.

The two servers are started automatically on startup by `supervisor`. If you are interested, take a look at [bootstrap.sh](./vnc_bootstrap/bootstrap.sh) and [supervisord.conf](./vnc_bootstrap/supervisord.conf) to see how we set the supervisor.

## Use VSCode to develop inside the container (Optional)

You can use the [VSCode](https://code.visualstudio.com/) and its [Remote-Containers](https://code.visualstudio.com/docs/remote/containers) extension to develop in the container. We provide a [.devcontainer/devcontainer.json](./.devcontainer/devcontainer.json) for you to start quickly.

1. Install [VSCode](https://code.visualstudio.com/) and its [Remote-Containers](https://code.visualstudio.com/docs/remote/containers) extension.

1. Copy the [.devcontainer](./.devcontainer) folder to your work folder, say `/home/peter/src`.

1. Open your work folder in VSCode using "**File-Open Folder**".

1. A notification will pop up in the right bottom corner asking you to reopen this folder in the container. Click "**Reopen in Container**". 

1. Now you can develop in the container.

**Note**: This method only works when the container and the VSCode are running on the same host. 

**Warning**: All the data in the container will be destroyed upon the container's closure, except those in your work folder. Do not store anything outside this folder if you want to use it later.

## Further reference

1. [Docker's Guide](https://docs.docker.com/)

1. [VSCode's Remote Development Guide for Containers](https://code.visualstudio.com/docs/remote/containers)

1. [Installing OpenRAVE on Ubuntu 16.04](https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html)

1. [Constrained Manipulation Planning Suite](https://sourceforge.net/projects/comps/)

1. [The Open Motion Planning Library](https://ompl.kavrakilab.org/)

1. [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

1. [Supervisor: A Process Control System](http://supervisord.org/)