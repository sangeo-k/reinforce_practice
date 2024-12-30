import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import String

class AnswerChecker(Node):
    def __init__(self):
        super().__init__('answer_checker')
        
        # /random_number 구독
        self.random_number_subscriber = self.create_subscription(Int32, '/random_number', self.set_random_number, 10)
        # /guess 구독
        self.guess_subscriber = self.create_subscription(Int32, '/guess', self.check_guess, 10)
        # /feedback 퍼블리시
        self.feedback_publisher = self.create_publisher(String, '/feedback', 10)
        # /number_received_signal 퍼블리시
        self.signal_publisher = self.create_publisher(Bool, '/number_received_signal', 10)
        self.random_number = None  # 현재 정답 값

    def set_random_number(self, msg):
        self.random_number = str(msg.data)  # 숫자 저장
        self.get_logger().info(f'Received random number: {self.random_number}')
        # 숫자 수신 신호를 퍼블리시
        self.signal_publisher.publish(Bool(data=True))

    def check_guess(self, msg):
        if self.random_number is None:
            self.get_logger().warning('No random number set yet.')
            return

        guess = str(msg.data)
        self.get_logger().info(f'Received guess: {guess}')

        # 스트라이크와 볼 계산
        strikes, balls = self.calculate_feedback(guess)

        # 피드백 퍼블리시
        feedback = f'{strikes}S{balls}B'
        self.feedback_publisher.publish(String(data=feedback))
        self.get_logger().info(f'Published feedback: {feedback}')

        # 정답이 맞으면 새로운 숫자 생성 신호
        if strikes == 4:  # 정답
            self.signal_publisher.publish(Bool(data=False))  # 새로운 숫자 생성 신호

    def calculate_feedback(self, guess):
        strikes = sum(1 for a, b in zip(self.random_number, guess) if a == b)
        balls = sum(1 for g in guess if g in self.random_number) - strikes
        return strikes, balls

def main(args=None):
    rclpy.init(args=args)
    node = AnswerChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
