FROM ubuntu:focal

ARG ROS_DISTRO=foxy
ARG ROS_PKG=desktop
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

# setup timezone
RUN echo 'Asia/Tokyo' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Asia/Tokyo /etc/localtime && \
    apt-get update && DEBIAN_FRONTEND=noninteractive && \
    apt-get install -q -y --no-install-recommends \
        tzdata && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# locale
RUN apt-get update && DEBIAN_FRONTEND=noninteractive && \
    apt-get install -q -y --no-install-recommends \
        locales && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8

# install basic packages
RUN apt-get update && DEBIAN_FRONTEND=noninteractive && \
    apt-get install -q -y --no-install-recommends \
        dirmngr \
        gnupg2 \
        lsb-release \
        curl \
        wget \
        git \
        vim \
        nano \
        gedit \
        build-essential \
        python3-dev \
        python3-pip && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

# setup sources.list
RUN echo "deb http://snapshots.ros.org/${ROS_DISTRO}/final/ubuntu focal main" > /etc/apt/sources.list.d/ros2-snapshots.list

# install ros2 packages
RUN apt-get update && DEBIAN_FRONTEND=noninteractive && \
    apt-get install -q -y --no-install-recommends \
        ros-${ROS_DISTRO}-${ROS_PKG}=0.9.2-1* \
        gazebo11 \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        ros-${ROS_DISTRO}-joint-state-publisher* \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-can-msgs \
        ros-${ROS_DISTRO}-rqt* \
        ros-${ROS_DISTRO}-simple-launch \
        ros-${ROS_DISTRO}-diagnostic* \
        python3-colcon-mixin \
        python3-rosdep \
        python3-vcstool && \
    rosdep init && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install xacro
RUN pip3 install xacro

# install pytorch
RUN pip3 install torch torchvision

# install other necessary packages
RUN apt-get update && DEBIAN_FRONTEND=noninteractive && \
    apt-get install -q -y --no-install-recommends \
        libgtk-3-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# add user
ARG USER_NAME=aiformula
ARG GROUP_NAME=aiformula
ARG UID=1000
ARG GID=1000
ARG PASSWORD=aiformula
RUN apt-get update && DEBIAN_FRONTEND=noninteractive && \
    apt-get install -q -y --no-install-recommends \
        sudo && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    groupadd -g ${GID} ${GROUP_NAME} && \
    useradd -u ${UID} -g ${GID} -G sudo -m -s /bin/bash ${USER_NAME} && \
    echo "${PASSWORD}:${PASSWORD}" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER ${USER_NAME}
# To avoid outputting an explanation about sudo when starting the terminal
RUN sudo echo ${USER_NAME}

ENV TERM=xterm-256color

RUN mkdir -pv ${HOME}/workspace/ros/src

# yolop
RUN git clone https://github.com/hustvl/YOLOP.git ${HOME}/workspace/YOLOP && \
    sed -i '/^scipy$/d' ${HOME}/workspace/YOLOP/requirements.txt && \
    pip3 install -r ${HOME}/workspace/YOLOP/requirements.txt
# ros2_socketcan
RUN git clone https://github.com/autowarefoundation/ros2_socketcan.git ${HOME}/workspace/ros/src/ros2_socketcan

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc && \
    echo "source ${HOME}/workspace/ros/install/setup.bash" >> ${HOME}/.bashrc && \
    echo "export PATH=\${PATH}:${HOME}/workspace/ros/src/aiformula/launchers/shellscript" >> ${HOME}/.bashrc

RUN echo "alias cb='cd ${HOME}/workspace/ros && colcon build --symlink-install --packages-up-to'" >> ${HOME}/.bash_aliases && \
    echo "alias cbcc='cd ${HOME}/workspace/ros && colcon build --cmake-clean-cache --symlink-install --packages-up-to'" >> ${HOME}/.bash_aliases && \
    echo "alias cc='rm -rf ${HOME}/workspace/ros/build ${HOME}/workspace/ros/install ${HOME}/workspace/ros/log && unset AMENT_PREFIX_PATH && unset CMAKE_PREFIX_PATH && source /opt/ros/${ROS_DISTRO}/setup.bash'" >> ${HOME}/.bash_aliases

CMD ["bash"]
