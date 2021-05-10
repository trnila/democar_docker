FROM ros:foxy

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install ros-foxy-desktop

RUN apt-get update && apt-get install -y \
  ros-foxy-rmw-cyclonedds-cpp \
  ros-foxy-vision-msgs \
  vim \
  strace \
  iproute2 \
  htop \
  wget \
  curl \
  gdb \
  tmux

ENV DISPLAY :0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN mkdir -p /opt/ws/

RUN apt-get install -y libyaml-cpp-dev libpcap-dev
RUN git clone https://github.com/ros/diagnostics.git /opt/ws/src/diagnostics -b foxy && cd /opt/ws && . /opt/ros/foxy/setup.sh && colcon build
RUN git clone https://github.com/ros-drivers/velodyne.git /opt/ws/src/velodyne -b ros2 && cd /opt/ws && . /opt/ros/foxy/setup.sh && colcon build
RUN git clone https://github.com/ros-perception/image_transport_plugins.git /opt/ws/src/image_transport_plugins -b ros2 && cd /opt/ws && . /opt/ros/foxy/setup.sh && colcon build
RUN git clone https://github.com/ros-perception/image_common.git /opt/ws/src/image_common -b ros2 && cd /opt/ws && . /opt/ros/foxy/setup.sh && colcon build
RUN git clone https://github.com/ros-perception/image_pipeline.git /opt/ws/src/image_pipeline -b ros2 && cd /opt/ws && . /opt/ros/foxy/setup.sh && colcon build


RUN mkdir -p /tools

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain nightly -y
ENV PATH=/root/.cargo/bin:$PATH
RUN apt-get install -y clang
RUN git clone https://github.com/eclipse-zenoh/zenoh-plugin-dds.git /tools/zenoh-plugin-dds && cd /tools/zenoh-plugin-dds/ && cargo build --release && cp ./target/release/zenoh-bridge-dds /usr/local/bin

RUN apt install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

ADD entry.sh /usr/local/bin/
ENTRYPOINT /usr/local/bin/entry.sh
