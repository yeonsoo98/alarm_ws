import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import time
import os

class SoundAlarmNode(Node):
    def __init__(self):
        super().__init__('sound_alarm_node')
        self.subscription = self.create_subscription(
            String,
            'alarm_trigger',
            self.alarm_callback,
            10
        )
        self.get_logger().info('알람 모드 실행.')
        pygame.mixer.init()  # Initialize the mixer module

    # 토픽에서 메세지 수신, 메세지 내용 로그에 기록
    def alarm_callback(self, msg):
        self.get_logger().info('Alarm triggered: "%s"' % msg.data)
        self.play_sound(msg.data)

    # 파일 경로 설정
    def play_sound(self, sound_name):
        sound_files = {
            "alarm1": "/home/ys/ros2_ws/stop_smoke.mp3",  # 절대 경로 사용
            "alarm2": "/home/ys/ros2_ws/focus_drive.mp3",  # 절대 경로 사용
        }
        if sound_name in sound_files:
            sound_path = sound_files[sound_name]
            if os.path.exists(sound_path):
                self.get_logger().info(f'Playing sound: {sound_path}')
                pygame.mixer.music.load(sound_path)
                pygame.mixer.music.play()
                # Wait for the sound to finish playing
                while pygame.mixer.music.get_busy():
                    time.sleep(1)
            else:
                self.get_logger().error(f'Sound file not found: {sound_path}')
        else:
            self.get_logger().warn('No sound file associated with the alarm: "%s"' % sound_name)

def main(args=None):
    rclpy.init(args=args)
    node = SoundAlarmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 topic pub /alarm_trigger std_msgs/msg/String "data: 'alarm1'"