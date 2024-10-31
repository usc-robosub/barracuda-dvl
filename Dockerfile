FROM ros:noetic-ros-base-focal
COPY . /opt/barracuda-dvl/
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    vim \
    && rm -rf /var/lib/apt/lists/*  \
    && mkdir /root/.ssh && chmod 700 /root/.ssh \
    && ssh-keyscan github.com >> /root/.ssh/known_hosts \
    && git config --global url."git@github.com:".insteadOf "https://github.com/" \
    && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "[ -f /opt/barracuda-dvl/catkin_ws/devel/setup.bash ] && source /opt/barracuda-dvl/catkin_ws/devel/setup.bash" >> /root/.bashrc \
    && echo "cd /opt/barracuda-dvl/catkin_ws" >> /root/.bashrc

WORKDIR /opt/barracuda-dvl/catkin_ws/
CMD ["/bin/bash"]
