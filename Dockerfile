FROM ubuntu:20.04

# Set noninteractive installation (to avoid getting stuck during building)
ARG DEBIAN_FRONTEND=noninteractive

# Update and install basic packages
RUN apt-get update && apt-get install -y \
    build-essential \
    python3 \
    python3-pip \
    git \
    curl \
    libompl-dev \
    ompl-demos \
    cmake \
    lsb-release

RUN mkdir -p /etc/apt/keyrings
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc
RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | \
    tee /etc/apt/sources.list.d/robotpkg.list

RUN apt-get update && apt-get install -qqy \
    robotpkg-py38-pinocchio \
    vim \
    libyaml-cpp-dev

ENV PATH "/opt/openrobots/bin:${PATH}"
ENV PKG_CONFIG_PATH "/opt/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH}"
ENV LD_LIBRARY_PATH "/opt/openrobots/lib:${LD_LIBRARY_PATH}"
ENV PYTHONPATH "/opt/openrobots/lib/python3.8/site-packages:${PYTHONPATH}"
ENV CMAKE_PREFIX_PATH "/opt/openrobots:${CMAKE_PREFIX_PATH}"

WORKDIR /home

RUN mkdir AnywareInterview
COPY . /home/AnywareInterview

# RUN git clone https://github.com/BolunDai0216/AnywareInterview.git

WORKDIR /home/AnywareInterview/cpp

RUN mkdir build

# Specify the command to run on container start
CMD ["bash"]