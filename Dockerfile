#Download base image ubuntu 18.04
FROM osrf/ros:melodic-desktop-full-bionic

# LABEL about the custom image
LABEL maintainer="lukasz.mitka@husarion.com"
LABEL description="This is custom Docker Image for robot_voice_recognition tutorial."

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

# Update Ubuntu Software repository
RUN apt update
RUN apt install -y dirmngr
RUN apt install -y apt-transport-https
RUN apt install -y lsb-release
RUN apt install -y ca-certificates
RUN apt remove -y cmdtest
RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg --output pubkey.gpg
RUN apt-key add pubkey.gpg

RUN echo "deb https://dl.yarnpkg.com/debian/ stable main" > /etc/apt/sources.list.d/yarn.list
RUN curl -sL https://deb.nodesource.com/setup_12.x --output setup.sh
RUN chmod a+x setup.sh
RUN ./setup.sh
RUN apt install -y nodejs
RUN apt install -y python-pip
RUN apt install -y openssl
RUN apt install -y ros-melodic-move-base
RUN apt install -y libserial-dev
RUN apt install -y yarn
RUN apt install -y net-tools
RUN apt install -y ros-melodic-grid-map
RUN apt install -y ros-melodic-joint-state-controller
RUN apt install -y ros-melodic-effort-controllers
RUN apt install -y ros-melodic-position-controllers
RUN apt install -y ros-melodic-gmapping
RUN apt install -y xdg-utils 
RUN apt install -y wget 
RUN apt install -y libxtst6 
RUN apt install -y libappindicator3-1

RUN rm -rf /var/lib/apt/lists/*

RUN useradd --create-home husarion
RUN mkdir -p /home/husarion

RUN echo "husarion:x:1000:1000:Developer,,,:/home/husarion:/bin/bash" >> /etc/passwd
RUN echo "husarion:x:1000:" >> /etc/group
RUN echo "husarion ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/husarion
RUN chmod 0440 /etc/sudoers.d/husarion

RUN chown husarion:husarion /home/husarion
USER husarion
ENV HOME /home/husarion
RUN mkdir -p /home/husarion/ros_ws/src

WORKDIR /home/husarion/ros_ws/src
RUN git clone https://github.com/husarion/robot_voice_control.git --single-branch --branch=fix_install
RUN git clone https://github.com/husarion/tutorial_pkg.git
RUN git clone https://github.com/husarion/rosbot_description.git

WORKDIR /home/husarion/ros_ws
RUN . /opt/ros/melodic/setup.sh && catkin_make
RUN . /home/husarion/ros_ws/devel/setup.sh && pip install -r $(rospack find voice_control)/../requirements.txt --user

RUN mkdir -p /home/husarion/ros_ws/src/robot_voice_control/voice_webserver/src/

WORKDIR /home/husarion/ros_ws/src/robot_voice_control/voice_webserver/src/
RUN yarn install

RUN curl -sSL https://github.com/mozilla/DeepSpeech/releases/download/v0.6.0/deepspeech-0.6.0-models.tar.gz --output deepspeech-0.6.0-models.tar.gz
RUN tar -xzf ./deepspeech-0.6.0-models.tar.gz
RUN mkdir -p /home/husarion/ros_ws/src/robot_voice_control/voice_webserver/src/cert/
WORKDIR /home/husarion/ros_ws/src/robot_voice_control/voice_webserver/src/cert/

RUN dd if=/dev/urandom of=/home/husarion/.rnd bs=256 count=1

RUN openssl req -newkey rsa:4096 -x509 -sha256 -days 3650 -nodes -out cert.crt -keyout key.key -subj "/C=PL/ST=Krakow/L=Krakow/O=Husarion/OU=Tutorials/CN=localhost"
RUN cat key.key cert.crt > server.pem
RUN sed -i 's/ENCRYPTED/RSA/g' server.pem

WORKDIR /home/husarion/ros_ws
