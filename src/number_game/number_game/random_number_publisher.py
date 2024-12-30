import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import random

class RandomNumberPublisher(Node):
    def __init__(self):
        super().__init__('random_number_publisher')
        self.publisher_ = self.create_publisher(Int32, '/random_number', 10)  # 숫자 퍼블리셔
        self.signal_subscriber = self.create_subscription(Bool, '/number_received_signal', self.handle_signal, 10)  # 숫자 수신 신호 구독
        self.active = True  # 숫자 생성 활성화 여부
        self.timer = self.create_timer(1.0, self.generate_new_number)  # 1초마다 숫자 생성 시도

    def generate_new_number(self):
        if self.active:
            random_number = random.randint(1000, 9999)  # 4자리 랜덤 숫자 생성
            self.publisher_.publish(Int32(data=random_number))
            self.get_logger().info(f'Published random number: {random_number}')

    def handle_signal(self, msg):
        if msg.data:  # True 신호를 받으면 숫자 생성을 멈춤
            self.active = False
            self.get_logger().info('AnswerChecker has received the random number. Stopping generation.')

def main(args=None):
    rclpy.init(args=args)
    node = RandomNumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
