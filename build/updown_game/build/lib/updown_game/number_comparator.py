import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String  # UP/DOWN 결과를 발행하기 위한 메시지 타입

class NumberComparator(Node):
    def __init__(self):
        super().__init__('number_comparator')
        self.target_subscription = self.create_subscription(Int32, 'target_number', self.set_target_number, 10)
        self.agent_subscription = self.create_subscription(Int32, 'agent_number', self.compare_number, 10)
        self.reward_publisher = self.create_publisher(Float32, 'reward', 10)
        self.result_publisher = self.create_publisher(String, 'up_down_result', 10)  # UP/DOWN 결과 발행
        self.number_received_publisher = self.create_publisher(Int32, 'number_received', 10)
        self.start_new_game_publisher = self.create_publisher(Int32, 'start_new_game', 10)
        self.is_active = False  # 숫자를 받았는지 상태 플래그
        self.silent_mode = True  # 초기 상태에서는 출력 억제
        self.target_number = None

    def set_target_number(self, msg):
        """NumberProvider에서 발행된 타겟 숫자 설정"""
        self.target_number = msg.data
        self.is_active = True  # 숫자를 받으면 활성화 상태로 전환
        self.silent_mode = False  # 숫자를 받으면 출력 활성화
        self.get_logger().info(f'Set target number: {self.target_number}')
        self.notify_number_received()

    def compare_number(self, msg):
        """Agent가 추측한 숫자를 타겟 숫자와 비교"""
        if not self.is_active:  # 활성화 상태가 아니면 동작하지 않음
            if not self.silent_mode:
                self.get_logger().warn("Comparator is inactive. Ignoring guesses.")
            return

        agent_number = msg.data
        result_msg = String()  # 결과 메시지 생성

        if agent_number == self.target_number:
            if not self.silent_mode:
                self.get_logger().info(f'Agent guessed correctly! Number: {agent_number}')
            self.reward_publisher.publish(Float32(data=1.0))  # 최대 보상
            result_msg.data = "CORRECT"
            self.result_publisher.publish(result_msg)  # CORRECT 결과 발행
            self.start_new_game()  # 새로운 게임 준비
        else:
            # UP 또는 DOWN 결과 설정 및 출력
            if agent_number < self.target_number:
                result_msg.data = "UP"
                if not self.silent_mode:
                    self.get_logger().info(f'UP! Agent guessed: {agent_number}')
            else:
                result_msg.data = "DOWN"
                if not self.silent_mode:
                    self.get_logger().info(f'DOWN! Agent guessed: {agent_number}')
            self.result_publisher.publish(result_msg)  # UP/DOWN 결과 발행

            # 거리 기반 보상 계산 및 발행
            reward = max(0.0, 1.0 - abs(agent_number - self.target_number) / 100.0)
            self.reward_publisher.publish(Float32(data=reward))

    def notify_number_received(self):
        """NumberProvider에 숫자가 처리되었음을 알림"""
        self.number_received_publisher.publish(Int32(data=1))

    def start_new_game(self):
        """새로운 게임 신호 발행"""
        self.start_new_game_publisher.publish(Int32(data=1))
        self.is_active = False  # 새로운 게임 시작 전 비활성화
        self.silent_mode = True  # 출력 억제 상태로 전환
        self.target_number = None  # 타겟 숫자 초기화

def main(args=None):
    rclpy.init(args=args)
    node = NumberComparator()
    rclpy.spin(node)
    rclpy.shutdown()
