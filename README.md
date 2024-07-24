# ROS 2 기반의 알람 모듈

ROS 2 (Robot Operating System 2)은 로봇 개발을 위한 오픈 소스 프레임워크로, 다양한 로봇 애플리케이션을 쉽게 구현할 수 있도록 지원. ROS 2를 기반으로 일정 메세지가 입력되면 해당하는 음성 MP3 파일이 재생되는 알람 모듈을 구현

## 준비사항

알람 모듈을 구현하기 위해서는 다음과 같은 준비가 필요:

1. ROS 2 설치
2. mp3 경로 구성
3. ROS 2 모듈 실행

## 구현 단계

### 1. ROS 2 노드 생성

먼저, ROS 2 노드를 생성. 이 노드는 일정 메시지를 구독(subscribe)하고, 해당 메시지가 들어오면 MP3 파일을 재생하는 역할을 합니다.

### 이제 알람 발행 노드가 주기적으로 알람 메시지를 발행하면, 알람 노드에서 해당 메시지를 받아 지정된 MP3 파일을 재생하게 됩니다.

### 런치 파일 생성 완료

#### 해당 시나리오 관련 mp3 파일 업로드

### 환경 구성
- Windows 환경 가능
- Linux 기반의 Jetson Orin
- MobiFren Bluetooth Speaker

## Docker 환경 
```
docker build -t ros2_humble_local:latest .
docker run -it --rm --name yskim_alarm --network host -v ~/ros2_ws:/ros2_ws ros2_humble_local:latest
```
### TO DO LIST
- Docker 환경 준비중
