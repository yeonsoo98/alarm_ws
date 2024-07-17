# 기본 ROS 2 Humble 이미지 사용
FROM osrf/ros:humble-desktop

# 필요한 패키지 설치
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pygame \
    pulseaudio \
    alsa-utils \
    && rm -rf /var/lib/apt/lists/*

# 레포지토리 코드 복사
COPY . /workspace

# 작업 디렉토리 설정
WORKDIR /workspace

# ROS 2 빌드
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select sound_alarm"

# 환경 설정을 위한 스크립트 작성
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# PulseAudio 설정
RUN mkdir -p /root/.config/pulse

# PulseAudio 서버를 시작하고 노드를 실행하는 스크립트 작성
RUN echo '#!/bin/bash\npulseaudio --start\nsource /opt/ros/humble/setup.bash\nsource /workspace/install/setup.bash\nros2 run sound_alarm sound_alarm_node' > /root/start.sh
RUN chmod +x /root/start.sh

# 엔트리포인트 설정
ENTRYPOINT ["/bin/bash", "-c", "/root/start.sh"]
