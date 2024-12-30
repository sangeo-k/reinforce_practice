import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
from itertools import permutations

class NumberGuessingAgent(Node):
    def __init__(self):
        super().__init__('number_guessing_agent')
        self.guess_publisher = self.create_publisher(Int32, '/guess', 10)
        self.feedback_subscriber = self.create_subscription(String, '/feedback', self.handle_feedback, 10)
        self.possible_numbers = [''.join(p) for p in permutations('0123456789', 4)]
        self.current_guess = None
        self.get_logger().info('Number Guessing Agent Node started')
        self.make_guess()

    def make_guess(self):
        # 가능한 숫자가 남아 있는 경우 추측
        if self.possible_numbers:
            self.current_guess = self.possible_numbers.pop(0)  # 후보에서 첫 번째 숫자 선택
            self.guess_publisher.publish(Int32(data=int(self.current_guess)))
            self.get_logger().info(f'Published guess: {self.current_guess}')
        else:
            # 가능한 숫자가 없는 경우 에러 로그 출력
            self.get_logger().error('No more possible numbers to guess.')

    def handle_feedback(self, msg):
        feedback = msg.data
        self.get_logger().info(f'Received feedback: {feedback}')

        # 피드백 분석
        strikes = int(feedback[0])
        balls = int(feedback[2])

        # 가능한 숫자 조합 업데이트
        self.update_possible_numbers(strikes, balls)

        # 다음 추측 수행
        self.make_guess()

    def update_possible_numbers(self, strikes, balls):
        self.get_logger().info(f'Filtering possible numbers for feedback: {strikes}S{balls}B')

        def is_possible(candidate):
            # 스트라이크 계산
            s = sum(1 for a, b in zip(candidate, self.current_guess) if a == b)
            # 볼 계산
            b = sum(1 for c in candidate if c in self.current_guess) - s
            # 피드백 조건과 일치하는지 확인
            return s == strikes and b == balls

        previous_count = len(self.possible_numbers)
        self.possible_numbers = [n for n in self.possible_numbers if is_possible(n)]
        updated_count = len(self.possible_numbers)

        # 디버깅 로그: 가능한 숫자 갱신 결과
        self.get_logger().info(f'Possible numbers updated: {updated_count} (from {previous_count})')

def main(args=None):
    rclpy.init(args=args)
    node = NumberGuessingAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
