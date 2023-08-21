FROM arturolaurenzi/xbot2_focal_base_nvidia:latest

USER root
SHELL ["/bin/bash", "-ic"]
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "--login", "-c"]

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt-get update --fix-missing && \
    apt-get install --no-install-recommends -y \
        nano \
        wget \
        ca-certificates \
        curl \
        sudo \
        libgtest-dev \
        libgmock-dev \
        python3-catkin-tools
#RUN apt-get autoremove -y && \
#    apt-get clean -y && \
#    rm -rf /var/lib/apt/lists/*

USER user
SHELL ["/bin/bash", "-ic"]

WORKDIR /home/user

# create forest ws and use it to clone and install CONCERT's simulation package
RUN mkdir tum_integration_ws
RUN mkdir tum_integration_ws/catkin_ws && \
    mkdir tum_integration_ws/build && \
    mkdir tum_integration_ws/install
WORKDIR /home/user/tum_integration_ws
ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https
########### src folder
RUN forest init

RUN forest add-recipes git@github.com:manuelvogel12/multidof_recipes.git --tag tum-concert-docker 
RUN forest grow tum_src --verbose --jobs 4 --pwd user
# EIGEN 3.4.0
WORKDIR /home/user/tum_integration_ws/src
# Pull and install eigen
RUN curl -LJO https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.bz2 && \
    tar -xvf eigen-3.4.0.tar.bz2 && \
    rm eigen-3.4.0.tar.bz2

# TODO USE THIS
WORKDIR /home/user/tum_integration_ws/build
RUN mkdir eigen-3.4.0
WORKDIR /home/user/tum_integration_ws/build/eigen-3.4.0
RUN cmake ../../src/eigen-3.4.0 -DCMAKE_INSTALL_PREFIX=/home/user/tum_integration_ws/install 
RUN make install -j8
# TODO REMOVE
# ENV EIGEN3_INCLUDE_DIR=/home/user/tum_integration_ws/src/eigen-3.4.0

########### catkin_ws folder
WORKDIR /home/user/tum_integration_ws/catkin_ws
RUN forest init
RUN forest add-recipes git@github.com:manuelvogel12/multidof_recipes.git --tag tum-concert-docker 
RUN forest grow tum_catkin_ws --verbose --jobs 4 --pwd user
# Install catkin packages
RUN echo "export ROBOT_RL_SIM_ROOT=/home/user/tum_integration_ws/catkin_ws" >> /home/user/.bashrc
ENV ROBOT_RL_SIM_ROOT=/home/user/tum_integration_ws/catkin_ws
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN catkin init && catkin config --install
# TODO: Uncomment this when the human-gazebo package is fixed
RUN catkin build concert_msgs  
# human-gazebo
RUN echo "source /home/user/tum_integration_ws/catkin_ws/devel/setup.bash" >> /home/user/.bashrc

# Install sara-shield
WORKDIR /home/user/tum_integration_ws/
RUN source setup.bash
WORKDIR /home/user/tum_integration_ws/build
RUN mkdir sara-shield
WORKDIR /home/user/tum_integration_ws/build/sara-shield
# RUN source /home/user/tum_integration_ws/catkin_ws/devel/setup.bash && source /home/user/tum_integration_ws/catkin_ws/setup.bash
RUN cmake ../../src/sara-shield/safety_shield -DCMAKE_INSTALL_PREFIX=/home/user/tum_integration_ws/install
RUN make install -j8
WORKDIR /home/user/tum_integration_ws/

RUN echo "source /home/user/tum_integration_ws/setup.bash" >> /home/user/.bashrc

# a few usage tips..
RUN echo 'echo "USAGE:"' >> /home/user/.bashrc
RUN echo 'echo "run simulation....: mon launch concert_gazebo concert.launch"' >> /home/user/.bashrc
RUN echo 'echo "run monitoring gui: xbot2-gui"' >> /home/user/.bashrc
RUN echo 'echo "run CartesI/O IK..: mon launch concert_cartesio concert.launch xbot:=true gui:=true"' >> /home/user/.bashrc
RUN echo 'echo "run odometry......: mon launch concert_odometry concert_odometry.launch publish_ground_truth:=true gui:=true"' >> /home/user/.bashrc

RUN echo 'echo ""' >> /home/user/.bashrc

# set ownership to user for the whole home folder
RUN chown -R user .
ENV DEBIAN_FRONTEND=dialog
# change user, copy start script (launches gazebo and gzweb)
USER user
