# NVIDIA Jetson 기반 Ubuntu 22.04 이미지 사용
FROM nvcr.io/nvidia/l4t-base:r35.1.0

# 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive

# 기본 패키지 업데이트 및 설치
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    locales \
    python3-pip \
    libasound2 \
    libasound2-dev \
    alsa-utils \
    pulseaudio \
    libpulse-dev \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt-get clean

# GPG 키 추가 및 ROS2 저장소 설정
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS2 Humble 및 필요한 패키지 설치
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    vim \
    gnupg \
    lsb-release \
    curl \
    ros-humble-desktop \
    python3-rosdep \
    python3-colcon-common-extensions \
    && apt-get clean

# rosdep 초기화 및 업데이트
RUN rosdep init && rosdep update

# pygame 설치
RUN pip3 install pygame

# ROS2 환경 설정
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 작업공간 생성 및 복사
WORKDIR /ros2_ws
COPY . /ros2_ws

# 빌드 파일 제거 및 종속성 설치
RUN rm -rf build/ install/ log/
RUN apt-get update
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep update"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# ROS2 작업공간 설정 자동화
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# 엔트리포인트 설정
ENTRYPOINT ["/bin/bash"]
