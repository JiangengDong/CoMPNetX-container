FROM nvidia/cudagl:10.0-devel-ubuntu16.04

RUN apt-get update && \
    apt-get install -y apt-utils sudo dialog wget lsb-release vim && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# install dependencies
RUN apt-get update && \ 
    apt-get install -y \
    cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy python-sympy qt4-dev-tools \
    libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev \
    libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev \
    libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev \
    libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev \
    libcairo2-dev libjasper-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev libccd-dev python-pip && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# work in /usr/local/src
WORKDIR /usr/local/src

# install collada-dom
RUN git clone https://github.com/rdiankov/collada-dom.git && \
    cd collada-dom && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install && \
    make clean
# install OpenSceneGraph
RUN git clone --branch OpenSceneGraph-3.4 https://github.com/openscenegraph/OpenSceneGraph.git && \
    cd OpenSceneGraph && \
    mkdir build && \
    cd build && \
    cmake .. -DDESIRED_QT_VERSION=4 && \
    make -j4 && \
    make install && \
    make clean
# install Flexible Collision Library
RUN git clone https://github.com/flexible-collision-library/fcl.git && \
    cd fcl && \
    git checkout 0.5.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install && \
    make clean
# install OpenRAVE
RUN git clone --branch latest_stable https://github.com/rdiankov/openrave.git && \
    cd openrave && \
    mkdir build && \
    cd build && \
    cmake .. -DOSG_DIR=/usr/local/lib64/ && \
    make -j4 && \
    make install && \
    make clean
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_:/usr/local/lib \
    PYTHONPATH=$PYTHONPATH:/usr/local/lib/python2.7/dist-packages/

# install ompl
RUN wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh && \
    chmod u+x install-ompl-ubuntu.sh && \
    ./install-ompl-ubuntu.sh && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/* && \
    ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen

# install ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get install -y ros-kinetic-desktop-full ros-kinetic-urdfdom-py && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/* && \
    rosdep init

# install cudnn
ENV CUDNN_VERSION 7.6.5.32
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"
RUN apt-get update && \
    apt-get install -y --no-install-recommends libcudnn7=$CUDNN_VERSION-1+cuda10.0 libcudnn7-dev=$CUDNN_VERSION-1+cuda10.0 && \
    apt-mark hold libcudnn7 && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# install pytorch
RUN apt-get update && \
    apt-get install -y libgoogle-glog-dev libgflags-dev unzip && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/* && \
    pip --no-cache-dir install --upgrade pip && \
    pip --no-cache-dir install future numpy ninja pyyaml mkl mkl-include setuptools cmake cffi pathlib glog typing pillow 
RUN git clone --recursive https://github.com/pytorch/pytorch
RUN cd pytorch && \
    python setup.py build
RUN cd pytorch && \
    python setup.py install
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/src/pytorch/torch/lib/ \
    CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/src/pytorch/

# add a user "atlas" with password "atlas"
RUN useradd -m -p fhOrBYoegyUZI atlas && \
    adduser atlas sudo
# change user and workdir
USER atlas
WORKDIR /home/atlas
SHELL ["/bin/bash", "-c"]

# install ROS packages
COPY --chown=atlas catkin_ws /home/atlas/catkin_ws
RUN rosdep update && \
    echo "source /opt/ros/kinetic/setup.bash" >> /home/atlas/.bashrc && \
    source /opt/ros/kinetic/setup.bash && \
    cd /home/atlas/catkin_ws && \
    catkin_make && \
    echo "source /home/atlas/catkin_ws/devel/setup.bash" >> /home/atlas/.bashrc

# add ormodels
COPY --chown=atlas ormodels /home/atlas/ormodels
ENV OPENRAVE_PLUGINS=/usr/local/share/openrave-0.9/plugins
ENV OPENRAVE_DATA=/home/atlas/ormodels
