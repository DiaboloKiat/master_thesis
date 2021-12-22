FROM nvidia/cuda:11.2.0-cudnn8-runtime-ubuntu20.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic
ARG GAZ=gazebo11

######################################## User ########################################

ENV SHELL=/bin/bash \
    NB_USER=kiat_thesis \
    NB_UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${NB_USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${NB_UID} \
    ${NB_USER} 

RUN echo "root:root" | chpasswd
RUN echo "${NB_USER}:111111" | chpasswd

######################################## Basic tools ########################################

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    vim \
    wget \
    sudo \
    cmake \
    gedit \
    gnupg2 \
    locales \
    dirmngr \
    mercurial \
    net-tools \
    lsb-release \
    openssh-client \
    build-essential \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libpcap-dev \
    libusb-1.0 \
    software-properties-common \
    ruby \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    python3-empy  \
    python3-opencv \
    python3-dbg \
    python3-venv \
    python3-scipy \
    python3-yaml \
    qtbase5-dev \
    unzip \
    git \
    udev \
    && add-apt-repository -y ppa:git-core/ppa \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Fixing TimeZone problem reported in VRX 2019
RUN export DEBIAN_FRONTEND=noninteractive \
    && apt update \
    && apt install -y \
        tzdata \
    && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && apt clean

######################################## ROS ########################################

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list \
    && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list

# setup keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# install packages
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install --no-install-recommends -y \
    ${GAZ} \
    lib${GAZ}-dev \
    ros-$ROS_DISTRO-desktop-full \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-gazebo-ros-control \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-geographic-info \
    ros-$ROS_DISTRO-hector-gazebo-plugins \
    ros-$ROS_DISTRO-hector-gazebo \
    ros-$ROS_DISTRO-hector-trajectory-server \
    ros-$ROS_DISTRO-joint-state-controller \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-teleop \
    ros-$ROS_DISTRO-kdl-parser-py \
    ros-$ROS_DISTRO-key-teleop \
    ros-$ROS_DISTRO-move-base \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-plugins \
    ros-$ROS_DISTRO-teleop-tools \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-tools \
    ros-$ROS_DISTRO-urdfdom-py \
    ros-$ROS_DISTRO-velodyne-gazebo-plugins \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-spacenav-node \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-velodyne-simulator \
    ros-$ROS_DISTRO-soem \
    ros-$ROS_DISTRO-openslam-gmapping \
    ros-$ROS_DISTRO-geodesy \
    ros-$ROS_DISTRO-ddynamic-reconfigure \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    ros-$ROS_DISTRO-rosserial-arduino \
    ros-$ROS_DISTRO-rosserial \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-perception \
    ros-$ROS_DISTRO-navigation \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-vcstool \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

######################################## PIP ########################################
RUN pip3 -V 

RUN pip3 install --upgrade pip setuptools

RUN pip3 install --upgrade \
    catkin-tools \
    gym \
    matplotlib \
    pandas \
    pypozyx \
    requests \
    jupyter \
    jupyter_http_over_ws \
    rospkg \
    scikit-image \
    scikit-learn \
    zerorpc \
    gdown

RUN pip3 install pyyaml \
    cython

RUN pip3 install \
    torch==1.8.0+cu111 \
    torchvision==0.9.0+cu111 \
    -f https://download.pytorch.org/whl/torch_stable.html

# tf 2.5.0 --> cuda 11.2
RUN pip3 install \
    tensorflow==2.5.0  \
    tensorflow-estimator==2.5.0 \
    tensorflow-probability==0.13.0 \
    trfl==1.1.0 \
    dm-acme==0.2.0 \
    dm-haiku==0.0.3 \
    dm-reverb==0.3.0 \
    dm-sonnet==2.0.0 \
    dm-tree==0.1.5

RUN pip3 install \
    dm-acme[jax]==0.2.0 \
    dm-acme[envs]==0.2.0 

RUN pip3 install -U 'jedi<0.18.0' \
    && jupyter serverextension enable --py jupyter_http_over_ws

######################################## PIP2 ########################################
RUN wget https://bootstrap.pypa.io/pip/2.7/get-pip.py \
    && python2 get-pip.py

RUN pip2 install --upgrade pip setuptools

RUN pip2 install \
    scipy

######################################## procman ########################################

RUN cd ${HOME} && git clone https://github.com/lcm-proj/lcm \
    && cd lcm \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

RUN cd ${HOME} && git clone https://github.com/DiaboloKiat/procman \
    && cd procman \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

######################################## realsense ########################################

RUN apt-get update && apt-get install -y \
    libsuitesparse-dev\
    software-properties-common \
    libusb-1.0-0-dev \
    libx11-dev \
    xorg-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# set the version of the realsense library
ENV LIBREALSENSE_VERSION 2.36.0
RUN wget https://github.com/IntelRealSense/librealsense/archive/v${LIBREALSENSE_VERSION}.tar.gz && \
    tar -xvzf v${LIBREALSENSE_VERSION}.tar.gz && \
    rm v${LIBREALSENSE_VERSION}.tar.gz && \
    mkdir -p librealsense-${LIBREALSENSE_VERSION}/build && \
    cd librealsense-${LIBREALSENSE_VERSION}/build && \
    cmake .. && \
    make && \
    make install && \
    rm -rf librealsense-${LIBREALSENSE_VERSION}

######################################## Setting ########################################

RUN chown -R ${NB_USER}:${NB_USER} ${HOME}/
RUN echo "${NB_USER} ALL=(ALL)  ALL" > /etc/sudoers

# Support of nvidia-docker 2.0
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

USER ${NB_USER}

WORKDIR ${HOME}
CMD ["bash"]