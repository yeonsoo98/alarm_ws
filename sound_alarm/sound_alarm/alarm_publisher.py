#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class AlarmPublisher(Node):
    def __init__(self):
        super().__init__('alarm_publisher')
        self.publisher_ = self.create_publisher(String, 'alarm_trigger', 10)
        self.timer = self.create_timer(1.0, self.publish_alarm)  # 1초마다 알람 발행
        self.get_logger().info('알람 발행 모드 실행.')

    def get_ai_generated_message(self):
        possible_messages = ['alarm1', 'alarm2', 'alarm3', 'alarm4','alarm5',
                             'alarm6', 'alarm7', 'alarm8', 'alarm9','alarm10']
        chosen_message = random.choice(possible_messages)
        self.get_logger().info(f'Generated message: {chosen_message}')
        return chosen_message

    def publish_alarm(self):
        msg = String()
        msg.data = self.get_ai_generated_message()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published alarm: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = AlarmPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
