FROM arturolaurenzi/xbot2_focal_base_nvidia:latest

USER root
SHELL ["/bin/bash", "-ic"]
RUN sudo apt-get update

USER user
SHELL ["/bin/bash", "-ic"]

WORKDIR /home/user

RUN echo "source $PWD/setup.bash" >> /home/user/.bashrc

# create forest ws and use it to clone and install CONCERT's simulation package
RUN mkdir tum_integration_ws
WORKDIR /home/user/tum_integration_ws
RUN mkdir src && mkdir build && mkdir install && mkdir catkin_ws
ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https

# src folder
WORKDIR /home/user/tum_integration_ws/src
RUN forest init
RUN forest add-recipes git@github.com:manuelvogel12/multidof_recipes.git --tag tum-concert-docker 
RUN forest grow tum_src --verbose --jobs 4 --pwd user
# catkin_ws folder
WORKDIR /home/user/tum_integration_ws/catkin_ws
RUN forest grow tum_catkin_ws --verbose --jobs 4 --pwd user

# a few usage tips..
RUN echo 'echo "USAGE:"' >> /home/user/.bashrc
RUN echo 'echo "run simulation....: mon launch concert_gazebo concert.launch"' >> /home/user/.bashrc
RUN echo 'echo "run monitoring gui: xbot2-gui"' >> /home/user/.bashrc
RUN echo 'echo "run CartesI/O IK..: mon launch concert_cartesio concert.launch xbot:=true gui:=true"' >> /home/user/.bashrc
RUN echo 'echo "run odometry......: mon launch concert_odometry concert_odometry.launch publish_ground_truth:=true gui:=true"' >> /home/user/.bashrc

RUN echo 'echo ""' >> /home/user/.bashrc

# set ownership to user for the whole home folder
RUN chown -R user .

# change user, copy start script (launches gazebo and gzweb)
USER user
