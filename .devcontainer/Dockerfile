FROM ros:humble-ros-base

RUN apt update && \
    apt upgrade -y && \
    apt install -y \
        ros-iron-ros2-control \
        ros-iron-ros2-controllers \
        ros-iron-xacro \
        ros-iron-teleop-twist-keyboard \
        gdb && \
    apt clean

ENTRYPOINT /ros_entrypoint.sh