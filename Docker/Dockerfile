FROM osrf/ros:noetic-desktop-focal

ARG ssh_prv_key

# install ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'

# install further ROS dependencies
RUN apt-get update && apt-get install -y \
	python3-pip\
        python3-rosdep  \
        python3-rosinstall  \
        python3-rosinstall-generator  \
        python3-wstool  \
        build-essential  \
        python3-rosdep \
        ros-noetic-catkin  \
        python3-catkin-tools \
	libtool \
        python-is-python3 \
        python3-pyqt5


#RUN apt-get install -y ros-noetic-octomap-rviz-plugins
	#autoconf \
	#automake \
	#ros-melodic-rotors-simulator

# install other packages
RUN apt-get update && apt-get install -y ssh\
        nano  \
        git \
        openssh-server

# authorize SSH Host
RUN mkdir -p /root/.ssh && \
    chmod 0700 /root/.ssh && \
    ssh-keyscan github.com > /root/.ssh/known_hosts

# add the keys and set permissions
RUN echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/id_rsa


# clone the git repository using SSH connection
WORKDIR /home
RUN git clone git@github.com:dp-tumLi/kkfly.git

# fetch data from submodules
WORKDIR /home/kkfly/

RUN git submodule init
RUN git submodule update

# remove SSH keys from docker
RUN rm -rf /root/.ssh/

# set working directory to project
WORKDIR /home/kkfly/catkin_ws/
RUN catkin config --extend /opt/ros/noetic
RUN catkin build

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN /bin/bash -c 'source devel/setup.bash'


