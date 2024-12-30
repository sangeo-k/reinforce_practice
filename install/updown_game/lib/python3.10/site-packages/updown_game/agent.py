import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String  # UP/DOWN 결과를 수신하기 위한 메시지 타입
from std_msgs.msg import Float32
import random


class Agent(Node):
    def __init__(self):
        super().__init__('agent')
        self.publisher = self.create_publisher(Int32, 'agent_number', 10)  # 추측 값 발행
        self.reward_subscription = self.create_subscription(Float32, 'reward', self.receive_reward, 10)
        self.result_subscription = self.create_subscription(String, 'up_down_result', self.receive_result, 10)  # UP/DOWN 결과 수신
        self.new_game_subscription = self.create_subscription(Int32, 'start_new_game', self.resume_after_number_received, 10)

        self.state = None
        self.action = None
        self.previous_action = None  # 이전 추측 값
        self.previous_result = None  # 이전 UP/DOWN 결과
        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.exploration_rate = 0.1
        self.q_table = {}
        self.guess_count = 0
        self.episode_count = 0
        self.active = True
        self.waiting_for_new_number = False  # 새로운 숫자를 기다리는 상태 플래그
        self.previous_guesses = set()  # 추측했던 숫자 기록
        self.repeated_guess_penalty = {}  # 숫자 반복에 따른 패널티 저장

        self.get_logger().info("Agent initialized. Starting first guess.")
        self.start_guess_loop(0.5)  # 추측 간격 0.5초 설정

    def start_guess_loop(self, delay):
        """추측 루프를 시작"""
        self.create_timer(delay, self.submit_guess)

    def submit_guess(self):
        """숫자를 추측"""
        if not self.active or self.waiting_for_new_number:
            return  # 비활성 상태 또는 새로운 숫자를 기다리는 상태에서는 동작하지 않음

        # 탐험 또는 활용 결정
        if random.random() < self.exploration_rate:
            while True:
                random_action = random.randint(1, 100)
                if random_action not in self.previous_guesses:
                    self.action = random_action
                    break
        else:
            self.action = self.get_best_action(self.state)  # 활용

        # 같은 숫자를 반복 추측하면 패널티를 적용
        if self.action in self.previous_guesses:
            penalty = self.calculate_penalty(self.action)
            if self.state is not None and self.action is not None:  # 상태와 행동이 유효한 경우만 업데이트
                self.q_table.setdefault(self.state, [0] * 101)  # Q-테이블 초기화
                self.q_table[self.state][self.action] -= self.learning_rate * penalty  # 패널티를 Q-값에 반영
            self.apply_penalty(-penalty)
            return

        # 추측 값을 발행
        self.publish_guess(self.action)
        self.previous_action = self.action
        self.previous_guesses.add(self.action)
        self.guess_count += 1
        self.get_logger().info(f"Submitted guess: {self.action} (Guess count: {self.guess_count})")


    def calculate_penalty(self, guess):
        """반복 추측에 따른 패널티 계산"""
        if guess not in self.repeated_guess_penalty:
            self.repeated_guess_penalty[guess] = 1.0  # 기본 패널티
        else:
            self.repeated_guess_penalty[guess] += 1.0  # 반복 추측 시 증가
        return self.repeated_guess_penalty[guess]  # 누적된 패널티 반환

    def publish_guess(self, guess):
        """추측 값을 number_comparator로 발행"""
        msg = Int32()
        msg.data = guess
        self.publisher.publish(msg)

    def receive_result(self, msg):
        """number_comparator로부터 UP/DOWN 결과 수신"""
        if self.waiting_for_new_number:
            return  # 새로운 숫자를 기다리는 동안 결과를 무시

        result = msg.data
        self.previous_result = result

        if result == "UP" and self.action <= self.previous_action:
            self.apply_penalty(-1.0)  # UP인데 낮은 숫자 추측 시 패널티
        elif result == "UP" and self.action > self.previous_action:
            self.apply_reward(1.0)  # UP인데 더 높은 숫자 추측 시 보상
        elif result == "DOWN" and self.action >= self.previous_action:
            self.apply_penalty(-1.0)  # DOWN인데 높은 숫자 추측 시 패널티
        elif result == "DOWN" and self.action < self.previous_action:
            self.apply_reward(1.0)  # DOWN인데 더 낮은 숫자 추측 시 보상

        self.get_logger().info(f"Received result: {result}")

    def apply_penalty(self, penalty_reward):
        """패널티를 적용"""
        self.receive_reward(Float32(data=penalty_reward))

    def apply_reward(self, reward):
        """보상을 적용"""
        self.receive_reward(Float32(data=reward))

    def receive_reward(self, msg):
        """보상 수신 및 Q-테이블 업데이트"""
        reward = msg.data
        self.get_logger().info(f"Received reward: {reward}")

        if self.state is not None and self.action is not None:
            self.update_q_table(self.state, self.action, reward)

        if reward == 1.0:  # 정답
            self.episode_count += 1
            self.get_logger().info(f"Correct guess! Ending episode {self.episode_count}. Total guesses: {self.guess_count}")
            self.active = False  # 추측 비활성화
            self.waiting_for_new_number = True  # 새로운 숫자를 기다리는 상태로 전환

    def resume_after_number_received(self, msg):
        """새로운 숫자를 받으면 작업 재개"""
        self.get_logger().info("New target number received. Resuming guesses.")
        self.waiting_for_new_number = False  # 대기 상태 해제
        self.reset_game()

    def reset_game(self):
        """게임 상태 초기화 및 새로운 에피소드 시작"""
        self.state = None
        self.action = None
        self.previous_action = None
        self.previous_result = None
        self.guess_count = 0
        self.previous_guesses.clear()
        self.repeated_guess_penalty.clear()  # 반복 패널티 초기화
        self.active = True

    def get_best_action(self, state):
        """Q-테이블에서 최적 행동 선택"""
        if state is None:  # 초기 상태 처리
            return random.randint(1, 100)  # 랜덤으로 시작
        if state not in self.q_table:
            self.q_table[state] = [0] * 101
        return max(range(1, 101), key=lambda x: self.q_table[state][x])

    def update_q_table(self, state, action, reward):
        """Q-테이블 업데이트"""
        if state not in self.q_table:
            self.q_table[state] = [0] * 101
        best_next_action = max(self.q_table[state])
        old_value = self.q_table[state][action]
        self.q_table[state][action] += self.learning_rate * (
            reward + self.discount_factor * best_next_action - old_value
        )
        self.get_logger().info(f"Updated Q-value for state {state}, action {action}: {old_value} -> {self.q_table[state][action]}")


def main(args=None):
    rclpy.init(args=args)
    node = Agent()
    rclpy.spin(node)
    rclpy.shutdown()
