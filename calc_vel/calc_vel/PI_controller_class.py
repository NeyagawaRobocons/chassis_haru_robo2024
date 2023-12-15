# description: PI制御を行うクラス

class PIController:
    def __init__(self, kp: float, ki: float, max_input: float):
        self.kp = kp
        self.ki = ki
        self.max_input = max_input
        self.integral = 0
        self.last_error = 0

    def update(self, target_position: float, current_position: float, dt: float) -> float:
        error = target_position - current_position
        self.integral += error * dt  # 積分計算にdtを考慮

        # 入力飽和のチェックと調整
        if self.calculate_input(error) > self.max_input or self.calculate_input(error) < -self.max_input:
            self.integral -= error * dt

        return self.calculate_input(error)

    def calculate_input(self, error: float) -> float:
        p_control = self.kp * error
        i_control = self.ki * self.integral
        control_input = p_control + i_control
        control_input = min(control_input, self.max_input)
        control_input = max(control_input, -self.max_input)

        return control_input

    def set_gains(self, kp: float, ki: float):
        """
        ゲインの設定を更新するメソッド
        """
        self.kp = kp
        self.ki = ki

    def reset_integral(self):
        """
        誤差の積分をリセットするメソッド
        """
        self.integral = 0