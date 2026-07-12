#!/usr/bin/env python3
"""为 LD08 的电机控制输入提供持续的 16 kHz、60% PWM。"""
import signal
import time

import Hobot.GPIO as GPIO


# GPIO.BOARD 编号，对应 RDK X5 40Pin 的物理针脚 33（PWM7）。
# 该针脚已用官方 simple_pwm.py 与 LD08 电机实际验证。
PWM_PIN = 33
PWM_FREQUENCY_HZ = 16_000
PWM_DUTY_CYCLE = 60.0


def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    pwm = GPIO.PWM(PWM_PIN, PWM_FREQUENCY_HZ)
    # 与 X5 官方示例的调用顺序保持一致。
    pwm.ChangeDutyCycle(PWM_DUTY_CYCLE)
    pwm.start(PWM_DUTY_CYCLE)
    print(
        f'LD08 PWM running: pin={PWM_PIN}, '
        f'frequency={PWM_FREQUENCY_HZ} Hz, duty={PWM_DUTY_CYCLE:.0f}%'
    )

    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while running:
            time.sleep(1.0)
    finally:
        pwm.stop()
        GPIO.cleanup()


if __name__ == '__main__':
    main()
