import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
import math

class ExamplePublisher(Node):
    def __init__(self):
        super().__init__('signal_generator')
        timer_period = 0.1
        self.pubisher = self.create_publisher(Float32, 'signal', 10)
        self.pubisher_counter = self.create_publisher(Float32, 'timer', 10)
        self.singal_generator = self.create_timer(timer_period, self.signal_callback)
        self.timer_generator = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Signal generator node initialized')
        self.msg = Float32()
        self.msg_counter = Float32()
        self.i = 0.0
    
    def timer_callback(self):
        self.msg_counter.data = self.i
        self.pubisher_counter.publish(self.msg_counter)
        self.i += 0.1

    def signal_callback(self):
        signal_value = math.sin(self.i)
        # self.get_logger().info('Signal value: {}')
        self.msg.data = signal_value
        self.pubisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    e_p = ExamplePublisher()
    rclpy.spin(e_p)
    e_p.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()