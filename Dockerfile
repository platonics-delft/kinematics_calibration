ARG ROS_DISTRO=humble
 
FROM osrf/ros:${ROS_DISTRO}-desktop

ENV ROS_DISTRO=${ROS_DISTRO}

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Delete existing user if it exists
RUN if getent passwd ${USER_UID}; then \
    userdel -r $(getent passwd ${USER_UID} | cut -d: -f1); \
    fi

# Delete existing group if it exists
RUN if getent group ${USER_GID}; then \
    groupdel $(getent group ${USER_GID} | cut -d: -f1); \
    fi

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


USER $USERNAME


SHELL ["/bin/bash", "-c"]
