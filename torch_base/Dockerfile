FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu16.04
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        apt-utils sudo dialog wget lsb-release vim cmake g++ git ipython minizip python-dev python-h5py \
        python-numpy python-scipy python-sympy qt4-dev-tools libgoogle-glog-dev libgflags-dev unzip python-pip && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*
RUN pip --no-cache-dir install --upgrade pip && \
    pip --no-cache-dir install setuptools && \
    pip --no-cache-dir install future numpy ninja pyyaml mkl mkl-include setuptools cmake cffi pathlib glog typing pillow
WORKDIR /usr/local/src
RUN git clone --recursive --branch v1.4.1 --depth 1 https://github.com/pytorch/pytorch
RUN cd pytorch && \
    python setup.py build
RUN cd pytorch && \
    python setup.py install
