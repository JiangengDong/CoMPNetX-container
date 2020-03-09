FROM nvidia/cudagl:10.1-runtime-ubuntu16.04

RUN apt-get update && \
    apt-get install -y apt-utils sudo dialog wget && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# add a user "atlas" with password "atlas"
RUN useradd -m -p fhOrBYoegyUZI atlas && \
    adduser atlas sudo && \
    mkdir -p /home/atlas/workspace

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

# work in atlas's home
WORKDIR /home/atlas/workspace

# install collada-dom
RUN git clone https://github.com/rdiankov/collada-dom.git && \
    cd collada-dom && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install
# install OpenSceneGraph
RUN git clone --branch OpenSceneGraph-3.4 https://github.com/openscenegraph/OpenSceneGraph.git && \
    cd OpenSceneGraph && \
    mkdir build && \
    cd build && \
    cmake .. -DDESIRED_QT_VERSION=4 && \
    make -j4 && \
    make install
# install Flexible Collision Library
RUN git clone https://github.com/flexible-collision-library/fcl.git && \
    cd fcl && \
    git checkout 0.5.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install
# install OpenRAVE
RUN git clone --branch latest_stable https://github.com/rdiankov/openrave.git && \
    cd openrave && \
    mkdir build && \
    cd build && \
    cmake .. -DOSG_DIR=/usr/local/lib64/ && \
    make -j4 && \
    make install
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_:/usr/local/lib \
    PYTHONPATH=$PYTHONPATH:/usr/local/lib/python2.7/dist-packages/

# install ompl
RUN apt-get update && \
    apt-get install -y libompl-dev ompl-demos && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# install ROS
RUN apt-get update && \
    apt-get install -y lsb-release && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get install -y ros-kinetic-desktop-full ros-kinetic-urdfdom-py && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/* && \
    rosdep init
    
# add a soft link for eigen
RUN ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen

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
COPY --chown=atlas ormodels /home/atlas/workspace/ormodels
ENV OPENRAVE_PLUGINS=/usr/local/share/openrave-0.9/plugins
ENV OPENRAVE_DATA=/home/atlas/workspace/ormodels