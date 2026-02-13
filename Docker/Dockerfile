FROM ros:noetic

WORKDIR /ws/src

RUN apt-get update 
RUN apt-get install python3-catkin-tools -y 
RUN apt-get install python3-pip -y
RUN apt-get install libbenchmark-dev -y
RUN apt-get install libeigen3-dev -y
RUN apt-get install python3-matplotlib -y
RUN apt-get install python3-pandas -y
RUN apt-get install gdb -y
RUN apt-get install git -y


WORKDIR /usr/include
RUN ln -sf eigen3/Eigen Eigen
RUN ln -sf eigen3/unsupported unsupported

WORKDIR /ws
RUN catkin config --extend /opt/ros/$ROS_DISTRO 
RUN catkin build 
