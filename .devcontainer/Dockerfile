FROM espressif/idf:release-v5.0

# Suppress interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -q && \
    apt-get install -yq sudo gosu nano lsb-release udev python3-pip && \
    rm -rf /var/lib/apt/lists/*

ENV TZ=Europe/Vienna
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN /bin/bash -c "source /opt/esp/idf/export.sh && \
    pip3 install catkin_pkg lark-parser colcon-common-extensions importlib-resources empy==3.3.4"

# Quiet the "Detecting Python" message each time you open a new shell
#   by setting "IDF_EXPORT_QUIET=1" and redirecting output to /dev/null
RUN echo "export IDF_EXPORT_QUIET=1" >> /root/.bashrc && \
    echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> /root/.bashrc

# Install additional tools
RUN apt install git -y

# Default to root user
USER root

# Default working directory inside container
WORKDIR /workspaces/firmware

CMD ["/bin/bash"]
