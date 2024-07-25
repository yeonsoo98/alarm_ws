#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import time
import os
os.environ['SDL_AUDIODRIVER'] = 'dsp'
pygame.mixer.init()  # Initialize the mixer module
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

    def alarm_callback(self, msg):
        self.get_logger().info('Alarm triggered: "%s"' % msg.data)
        self.play_sound(msg.data)

    def play_sound(self, sound_name):
        sound_files = {
            "alarm1": "/home/ys/ros2_ws/stop_smoke.mp3",
            "alarm2": "/home/ys/ros2_ws/care_pet.mp3",
            "alarm3": "/home/ys/ros2_ws/focus_drive.mp3",
            "alarm4": "/home/ys/ros2_ws/need_break.mp3",
            "alarm5": "/home/ys/ros2_ws/stop_drive.mp3",
            "alarm6": "/home/ys/ros2_ws/stop_eat.mp3",
            "alarm7": "/home/ys/ros2_ws/stop_eat_driving.mp3",
            "alarm8": "/home/ys/ros2_ws/stop_fight.mp3",
            "alarm9": "/home/ys/ros2_ws/stop_phone.mp3",
            "alarm10": "/home/ys/ros2_ws/use_seatbelt.mp3",
        }
        if sound_name in sound_files:
            sound_path = sound_files[sound_name]
            self.get_logger().info(f'Attempting to play sound: {sound_path}')
            if os.path.exists(sound_path):
                self.get_logger().info(f'Playing sound: {sound_path}')
                pygame.mixer.music.load(sound_path)
                pygame.mixer.music.play()
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
