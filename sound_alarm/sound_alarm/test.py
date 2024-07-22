#!/usr/bin/env python3

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
        self.current_alarm = None  # 현재 재생 중인 알람
        self.alarm_priorities = {
            "alarm1": 1,
            "alarm2": 2,
            "alarm3": 3,
            "alarm4": 4,
            "alarm5": 5,
            "alarm6": 6,
            "alarm7": 7,
            "alarm8": 8,
        }

    def alarm_callback(self, msg):
        self.get_logger().info('Alarm triggered: "%s"' % msg.data)
        self.handle_alarm(msg.data)

    def handle_alarm(self, sound_name):
        if (self.current_alarm is None or 
            self.alarm_priorities.get(sound_name, 0) > self.alarm_priorities.get(self.current_alarm, 0)):
            self.current_alarm = sound_name
            self.play_sound(sound_name)

    def play_sound(self, sound_name):
        sound_files = {
            "alarm1": "/home/ys/ros2_ws/stop_smoke.mp3",  # 
            "alarm2": "/home/ys/ros2_ws/focus_drive.mp3",  # 
            "alarm2": "/home/ys/ros2_ws/care_pet.mp3",  # 
            "alarm2": "/home/ys/ros2_ws/need_break.mp3",  # 
            "alarm2": "/home/ys/ros2_ws/stop_drive.mp3",  # 
            "alarm2": "/home/ys/ros2_ws/stop_eat.mp3",  # 
            "alarm2": "/home/ys/ros2_ws/stop_eat_driving.mp3",  # 
            "alarm2": "/home/ys/ros2_ws/stop_fight.mp3",  # 
        }
        if sound_name in sound_files:
            sound_path = sound_files[sound_name]
            if os.path.exists(sound_path):
                self.get_logger().info(f'Playing sound: {sound_path}')
                pygame.mixer.music.load(sound_path)
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy():
                    time.sleep(1)
                self.current_alarm = None  # 알람 재생이 끝나면 현재 알람 초기화
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
# ros2 topic pub /alarm_trigger std_msgs/msg/String "data: 'alarm2'"