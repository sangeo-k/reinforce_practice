import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class NumberProvider(Node):
    def __init__(self):
        super().__init__('number_provider')
        self.publisher = self.create_publisher(Int32, 'target_number', 10)
        self.subscriber = self.create_subscription(Int32, 'number_received', self.stop_publishing, 10)
        self.new_game_subscriber = self.create_subscription(Int32, 'start_new_game', self.start_new_game, 10)
        self.timer = self.create_timer(1.0, self.publish_random_number)  # 1초 간격 타이머
        self.current_target = None
        self.is_active = True  # 숫자 발행 활성화 여부

    def publish_random_number(self):
        """1초 간격으로 랜덤 숫자를 발행"""
        if self.is_active:
            self.current_target = self.generate_random_number()
            msg = Int32()
            msg.data = self.current_target
            self.publisher.publish(msg)
            self.get_logger().info(f'Published target number: {self.current_target}')

    def stop_publishing(self, msg):
        """NumberComparator가 숫자를 받으면 발행 멈춤"""
        if msg.data == 1:
            self.is_active = False
            self.get_logger().info("Number received by NumberComparator. Stopping publishing.")

    def start_new_game(self, msg):
        """새로운 게임 시작 신호를 받으면 발행 재개"""
        if msg.data == 1:
            self.is_active = True
            self.get_logger().info("Starting a new game. Resuming number publishing.")

    def generate_random_number(self):
        """랜덤 숫자 생성"""
        import random
        return random.randint(1, 100)

def main(args=None):
    rclpy.init(args=args)
    node = NumberProvider()
    rclpy.spin(node)
    rclpy.shutdown()
