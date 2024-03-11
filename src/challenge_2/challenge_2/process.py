import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
import math as mt
import numpy as np

class ExampleSubscriber(Node):
    def __init__(self):
        super().__init__('process_node')
        self.get_logger().info('Listener node initialized')

        self.sub = self.create_subscription(Float32, 'signal', self.listener_callback, 10)
        self.sub_timer = self.create_subscription(Float32, 'timer', self.listener_timer_callback, 10)
        self.singal_moded = self.create_subscription(Float32, 'signal', self.listener_process_callback, 10)
        self.signal_shift = self.create_subscription(Float32, 'timer', self.listener_process_shift_callback, 10)
        self.signal_circle = self.create_subscription(Float32, 'timer', self.listener_process_circle_callback, 10)


        self.modified_signal_pub = self.create_publisher(Float32, 'modified_signal', 10)
        self.modified_signal_pub_2 = self.create_publisher(Float32, 'modified_signal_2', 10)
        self.modified_signal_pub_3 = self.create_publisher(Float32, 'proc_signal', 10)
        self.modified_signal_pub_4 = self.create_publisher(Float32, 'proc_signal_2', 10)

        time_shift = 1.5

    def listener_callback(self, msg):
        self.get_logger().info('Signal value: {}'.format(msg.data))

    def listener_timer_callback(self, msg):
        self.get_logger().info('Time: {}'.format(msg.data))
    
    def listener_process_callback(self, msg):
        modifed_signal_1 = (msg.data + 1)
        self.get_logger().info('Offset signal value: {}'.format(modifed_signal_1))
        # self.modified_signal_pub.publish(Float32(data=modifed_signal_1))
        
        modified_signal_2 = (msg.data * 0.5)
        self.get_logger().info('Half amplitude signal value: {}'.format(modified_signal_2))
        # self.modified_signal_pub_2.publish(Float32(data=modified_signal_2))

    def listener_process_shift_callback(self,msg):
        time_shift = 0.35
        signal_shifting = mt.sin(msg.data+time_shift)
        signal_modification = (signal_shifting + 1)*0.5
        modified_signal_shift = self.get_logger().info('Modified signal: {}'.format(signal_modification))
        self.modified_signal_pub_3.publish(Float32(data=signal_modification))

    def listener_process_circle_callback(self,msg):
        signal_circle_formula = mt.sin(msg.data)**2 + mt.cos(msg.data)**2
        modified_signal_circle = self.get_logger().info('Circle formula signal: {}'.format(signal_circle_formula))
        self.modified_signal_pub_4.publish(Float32(data=signal_circle_formula))

def main(args=None):
    rclpy.init(args=args)
    e_s = ExampleSubscriber()
    rclpy.spin(e_s)
    e_s.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()