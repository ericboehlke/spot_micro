FROM ros:foxy-ros-base-l4t-r32.4.4
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
RUN apt-get update && apt-get install -y \
    tmux \
    zsh \
    curl \
    wget \
    vim \
    sudo \
    && rm -rf /var/likb/apt/lists/*
RUN pip3 install adafruit-circuitpython-servokit Jetson.GPIO

COPY 99-gpio.rules /etc/udev/rules.d/99-gpio.rules

ARG USER=spot
ARG UID=1000
ARG GID=1000
ARG PW=micro

RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PW}" | \
    chpasswd && \
    groupadd -f -r gpio && \ 
    groupmod --gid=108 i2c && \ 
    usermod -a -G gpio ${USER} && \
    usermod -a -G i2c ${USER} && \
    usermod -a -G sudo ${USER} 

USER ${UID}:${GID}
WORKDIR /home/${USER}

ENTRYPOINT ["./ros2_ws/src/spot_micro/entrypoint.sh"]
CMD ["bash"]
