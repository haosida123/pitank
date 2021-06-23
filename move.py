#!/usr/bin/env python3
# File name   : move.py
# Description : Control Motor
# Product     : GWR
# Website     : www.gewbot.com
# Author      : William
# Date        : 2019/07/24
import time
import RPi.GPIO as GPIO
import RPIservo
import argparse
import json
import os

# motor_EN_A: Pin7  |  motor_EN_B: Pin11
# motor_A:  Pin8,Pin10    |  motor_B: Pin13,Pin12

# Motor_A_EN    = 4
# Motor_B_EN    = 17

# Motor_A_Pin1  = 14
# Motor_A_Pin2  = 15
# Motor_B_Pin1  = 27
# Motor_B_Pin2  = 18
Motor_B_EN    = 4
Motor_A_EN    = 17

Motor_B_Pin1  = 14
Motor_B_Pin2  = 15
Motor_A_Pin1  = 27
Motor_A_Pin2  = 18

# Dir_forward   = 1
# Dir_backward  = 0
Dir_forward   = 0
Dir_backward  = 1

left_forward  = 0
left_backward = 1

right_forward = 0
right_backward= 1

pwn_A = 0
pwm_B = 0

def motorStop():#Motor stops
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_B_Pin1, GPIO.LOW)
    GPIO.output(Motor_B_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)
    GPIO.output(Motor_B_EN, GPIO.LOW)


def setup():#Motor initialization
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_B_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(Motor_B_Pin1, GPIO.OUT)
    GPIO.setup(Motor_B_Pin2, GPIO.OUT)

    motorStop()
    try:
        pwm_A = GPIO.PWM(Motor_A_EN, 1000)
        pwm_B = GPIO.PWM(Motor_B_EN, 1000)
    except:
        pass


def motor_left(status, direction, speed):#Motor 2 positive and negative rotation
    if status == 0: # stop
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)
    else:
        if direction == Dir_backward:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)
        elif direction == Dir_forward:
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed):#Motor 1 positive and negative rotation
    if status == 0: # stop
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
    else:
        if direction == Dir_forward:#
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        elif direction == Dir_backward:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
    return direction


def destroy():
    motorStop()
    GPIO.cleanup()             # Release resource


class Move(object):
    def __init__(self) -> None:
        setup()
        self.speed = 100
        self.servo = RPIservo.ServoCtrl()
        self.servo.start()
        if not os.path.isfile("servostate.json"):
            self.servo.moveInit()
            with open("servostate.json", 'w') as f:
                json.dump(self.servo.nowPos, f)
        else:
            with open("servostate.json", 'r') as f:
                self.servo.initPos = json.load(f)
            self.servo.moveInit()


    def move(self, direction, turn, radius=0.6):   # 0 < radius <= 1  
        speed = self.speed
        if direction == 'forward':
            if turn == 'right':
                motor_left(0, left_backward, int(speed*radius))
                motor_right(1, right_forward, speed)
            elif turn == 'left':
                motor_left(1, left_forward, speed)
                motor_right(0, right_backward, int(speed*radius))
            else:
                motor_left(1, left_forward, speed)
                motor_right(1, right_forward, speed)
        elif direction == 'backward':
            if turn == 'right':
                motor_left(0, left_forward, int(speed*radius))
                motor_right(1, right_backward, speed)
            elif turn == 'left':
                motor_left(1, left_backward, speed)
                motor_right(0, right_forward, int(speed*radius))
            else:
                motor_left(1, left_backward, speed)
                motor_right(1, right_backward, speed)
        elif direction == 'no':
            if turn == 'right':
                motor_left(1, left_backward, speed)
                motor_right(1, right_forward, speed)
            elif turn == 'left':
                motor_left(1, left_forward, speed)
                motor_right(1, right_backward, speed)
            else:
                motorStop()
        else:
            pass
    
    def stop(self):
        motorStop()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        # def __exit__(self, exception_type, exception_value, traceback):
        destroy()
        with open("servostate.json", 'w') as f:
            json.dump(self.servo.nowPos, f)
        self.servo.initPos = self.servo.nowPos
        self.servo.stop()
        # print(f"Move({args}) __exit__ called")


def test():
    movetime = 0.1
    with Move() as move:
        move.move('forward', 'no')
        time.sleep(movetime)
        move.stop()
    return
    try:
        # speed_set = 40
        # speed_set = 60
        speed_set = 100
        # setup()
        # move(speed_set, 'forward', 'no', 0.8)
        # time.sleep(movetime)
        # move(speed_set, 'forward', 'left', 0.8)
        # time.sleep(movetime)
        # move(speed_set, 'backward', 'right', 0.8)
        # time.sleep(movetime)
        # move(speed_set, 'no', 'left', 0.8)
        # time.sleep(movetime)
        # move(speed_set, 'no', 'right', 0.8)
        # time.sleep(movetime)

        # GPIO.output(Motor_B_Pin1, GPIO.LOW)
        # GPIO.output(Motor_B_Pin2, GPIO.HIGH)
        # pwm_B.start(0)
        # pwm_B.ChangeDutyCycle(40)
        # time.sleep(movetime)
        # pwm_B.start(100)
        # pwm_B.ChangeDutyCycle(40)
        # time.sleep(movetime)

        motorStop()
        destroy()
    except KeyboardInterrupt:
        destroy()


if __name__ == '__main__':
    # test()
    parser = argparse.ArgumentParser()
    parser.add_argument("direction", type=str)
    parser.add_argument("-time", type=float, default=0.5)
    parser.add_argument("-speed", type=float, default=100)
    args = parser.parse_args()

    movetime = args.time
    direct = args.direction

    with Move() as move:
        move.speed = args.speed

        if direct == "forward":
            move.move('forward', 'no')
        elif direct == "backward":
            move.move('backward', 'no')
        elif direct == "right":
            move.move('no', 'right')
        elif direct == "left":
            move.move('no', 'left')
        elif direct == "up":
            move.servo.move("camera", "up")
        elif direct == "down":
            move.servo.move("camera", "down")

        time.sleep(movetime)
        move.servo.stopWiggle()
        move.stop()
