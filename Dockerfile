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
RUN echo "source $PWD/setup.bash" >> /home/user/.bashrc
ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https

RUN forest init
RUN forest add-recipes git@github.com:manuelvogel12/multidof_recipes.git --tag master
RUN forest grow pybind11 --verbose --jobs 4 --pwd user


########### catkin_ws folder
WORKDIR /home/user/tum_integration_ws/catkin_ws
RUN forest init
RUN forest add-recipes git@github.com:manuelvogel12/multidof_recipes.git --tag master 
RUN forest grow tum_catkin_ws --verbose --jobs 4 --pwd user
RUN catkin build

RUN echo "source /home/user/tum_integration_ws/catkin_ws/devel/setup.bash" >> /home/user/.bashrc
RUN echo "source /home/user/tum_integration_ws/setup.bash" >> /home/user/.bashrc

########### src folder
WORKDIR /home/user/tum_integration_ws
RUN forest grow tum_src --verbose --jobs 4 --pwd user


# a few usage tips..
RUN echo 'echo "USAGE:"' >> /home/user/.bashrc
RUN echo 'echo "run simulation....: mon launch sara_shield concert.launch"' >> /home/user/.bashrc
RUN echo 'echo "run monitoring gui: xbot2-gui"' >> /home/user/.bashrc
RUN echo 'echo "run rviz..........: rviz -d $(rospack find sara_shield)/safety_shield/rviz/concert_sara_shield.rviz"' >> /home/user/.bashrc

RUN echo 'echo ""' >> /home/user/.bashrc

# set ownership to user for the whole home folder
RUN chown -R user .
ENV DEBIAN_FRONTEND=dialog
# change user, copy start script (launches gazebo and gzweb)
USER user
