FROM jeguzzi/ros:armhf-kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    ros-kinetic-tf2-ros \
    ros-kinetic-tf \
    ros-kinetic-diagnostics \
    python-serial

RUN git clone https://github.com/pozyxLabs/Pozyx-Python-library.git /tmp/pozyx
RUN /bin/bash -c 'pushd /tmp/pozyx; python setup.py install; popd;'

COPY . src/pozyx_ros_driver
RUN catkin build
