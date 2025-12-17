FROM ros:humble
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV ROS_DOMAIN_ID=42

SHELL ["/bin/bash", "-c"]

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3 python3-pip
# Add code to image
ADD . /stretch-ws/src
# Configure rosdep
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install -y --from-paths /stretch-ws --ignore-src --rosdistro=${ROS_DISTRO}
# Build
WORKDIR /stretch-ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install
WORKDIR /
# Install mujoco sim
WORKDIR /stretch-ws/src
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
RUN git clone https://github.com/hello-robot/stretch_mujoco --recurse-submodules
WORKDIR /stretch-ws/src/stretch_mujoco
# Source ROS on terminal start
RUN echo "source /stretch-ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

CMD [ "bash" ]