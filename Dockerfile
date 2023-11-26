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
    ompl-demos

RUN apt-get install -qqy \
    cmake \
    lsb-release

# Specify the command to run on container start
CMD ["bash"]