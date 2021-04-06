#!/usr/bin/env python3
# File name   : servo.py
# Description : Control Servos
# Author      : William
# Date        : 2019/02/23
from __future__ import division
from os import kill
import time
import RPi.GPIO as GPIO
import sys
import Adafruit_PCA9685
import threading

import random
'''
change this form 1 to -1 to reverse servos
'''
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

init_pwm0 = 300
init_pwm1 = 300
init_pwm2 = 300
init_pwm3 = 300

init_pwm4 = 300
init_pwm5 = 300
init_pwm6 = 300
init_pwm7 = 300

init_pwm8 = 300
init_pwm9 = 300
init_pwm10 = 300

init_pwm11 = 300
init_pwm12 = 490
init_pwm13 = 430
# init_pwm14 = 300
init_pwm14 = 350
init_pwm15 = 300
# init_pwm11 = 300
# init_pwm12 = 300
# init_pwm13 = 300
# init_pwm14 = 300
# init_pwm15 = 300


class ServoCtrl(threading.Thread):

    def __init__(self, *args, **kwargs):
        self.sc_direction = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        self.initPos = [init_pwm0, init_pwm1, init_pwm2, init_pwm3,
                        init_pwm4, init_pwm5, init_pwm6, init_pwm7,
                        init_pwm8, init_pwm9, init_pwm10, init_pwm11,
                        init_pwm12, init_pwm13, init_pwm14, init_pwm15]
        self.goalPos = [p for p in self.initPos]
        self.nowPos = [p for p in self.initPos]
        self.bufferPos = [p for p in self.initPos]
        self.lastPos = [p for p in self.initPos]
        self.ingGoal = [p for p in self.initPos]
        self.maxPos = [560, 560, 560, 560, 560, 560, 560,
            560, 560, 560, 560, 320, 560, 560, 560, 560]
        self.minPos = [100, 100, 100, 100, 100, 100, 100,
            100, 100, 100, 100, 150, 100, 100, 100, 100]
        self.scSpeed = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.ctrlRangeMax = 560
        self.ctrlRangeMin = 100
        self.angleRange = 180

        '''
        scMode: 'init' 'auto' 'certain' 'quick' 'wiggle'
        '''
        self.scMode = 'auto'
        # self.scTime = 2.0
        self.scTime = 0.0
        self.scSteps = 30
        # self.scSteps = 15

        self.scDelay = 0.037
        self.scMoveTime = 0.037

        self.goalUpdate = 0
        self.wiggleID = 0
        self.wiggleDirection = 1

        self.stop_flag = False
        self.executing = False
        super(ServoCtrl, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    def pause(self):
        # print('......................pause..........................')
        self.__flag.clear()

    def resume(self):
        # print('resume')
        self.__flag.set()

    def moveInit(self):
        self.scMode = 'init'
        for i in range(0, 16):
            # pwm.set_pwm(i, 0, self.initPos[i])
            self.lastPos[i] = self.initPos[i]
            self.nowPos[i] = self.initPos[i]
            self.bufferPos[i] = float(self.initPos[i])
            self.goalPos[i] = self.initPos[i]
        # print('initializing')
        self.moveAuto()
        self.pause()

    def initConfig(self, ID, initInput, moveTo):
        if initInput > self.minPos[ID] and initInput < self.maxPos[ID]:
            self.initPos[ID] = initInput
            if moveTo:
                pwm.set_pwm(ID, 0, self.initPos[ID])
        else:
            print('initPos Value Error.')

    def moveServoInit(self, ID):
        self.scMode = 'init'
        # self.scMode = 'auto'
        for i in range(0, len(ID)):
            # pwm.set_pwm(ID[i], 0, self.initPos[ID[i]])
            self.lastPos[ID[i]] = self.initPos[ID[i]]
            self.nowPos[ID[i]] = self.initPos[ID[i]]
            self.bufferPos[ID[i]] = float(self.initPos[ID[i]])
            self.goalPos[ID[i]] = self.initPos[ID[i]]
        self.moveAuto()
        self.pause()

    def posUpdate(self):
        self.goalUpdate = 1
        for i in range(0, 16):
            self.lastPos[i] = self.nowPos[i]
        self.goalUpdate = 0

    def speedUpdate(self, IDinput, speedInput):
        for i in range(0, len(IDinput)):
            self.scSpeed[IDinput[i]] = speedInput[i]

    def moveAuto(self):
        self.pause()
        self.executing = True
        for i in range(0, 16):
            self.ingGoal[i] = self.goalPos[i]

        for i in range(0, self.scSteps):
            for dc in range(0, 16):
                while self.goalUpdate:
                    time.sleep(0.01)
                self.nowPos[dc] = int(round(
                    (self.lastPos[dc] + (((self.goalPos[dc] - self.lastPos[dc])/self.scSteps)*(i+1))), 0))
                pwm.set_pwm(dc, 0, self.nowPos[dc])

                if self.ingGoal != self.goalPos:
                    self.posUpdate()
                    time.sleep(self.scTime/self.scSteps)
                    return 1
            # print(f"now:{self.nowPos[12]}", end=',')
            time.sleep((self.scTime/self.scSteps))
            # time.sleep((self.scTime/self.scSteps - self.scMoveTime))

        self.posUpdate()
        self.executing = False
        return 0

    def moveCert(self):
        for i in range(0, 16):
            self.ingGoal[i] = self.goalPos[i]
            self.bufferPos[i] = self.lastPos[i]

        while self.nowPos != self.goalPos:
            for i in range(0, 16):
                if self.lastPos[i] < self.goalPos[i]:
                    self.bufferPos[i] += self.pwmGenOut(self.scSpeed[i])/(1/self.scDelay)
                    newNow = int(round(self.bufferPos[i], 0))
                    if newNow > self.goalPos[i]: newNow = self.goalPos[i]
                    self.nowPos[i] = newNow
                elif self.lastPos[i] > self.goalPos[i]:
                    self.bufferPos[i] -= self.pwmGenOut(self.scSpeed[i])/(1/self.scDelay)
                    newNow = int(round(self.bufferPos[i], 0))
                    if newNow < self.goalPos[i]: newNow = self.goalPos[i]
                    self.nowPos[i] = newNow

                if not self.goalUpdate:
                    pwm.set_pwm(i, 0, self.nowPos[i])

                if self.ingGoal != self.goalPos:
                    self.posUpdate()
                    return 1
            self.posUpdate()
            time.sleep(self.scDelay-self.scMoveTime)

        else:
            self.pause()
            return 0

    def pwmGenOut(self, angleInput):
        return int(round(((self.ctrlRangeMax-self.ctrlRangeMin)/self.angleRange*angleInput), 0))

    def setAutoTime(self, autoSpeedSet):
        self.scTime = autoSpeedSet

    def setDelay(self, delaySet):
        self.scDelay = delaySet

    def autoSpeed(self, ID, angleInput):
        self.scMode = 'auto'
        self.goalUpdate = 1
        for i in range(0, len(ID)):
            newGoal = self.initPos[ID[i]] + \
                self.pwmGenOut(angleInput[i])*self.sc_direction[ID[i]]
            if newGoal > self.maxPos[ID[i]]: newGoal = self.maxPos[ID[i]]
            elif newGoal < self.minPos[ID[i]]: newGoal = self.minPos[ID[i]]
            self.goalPos[ID[i]] = newGoal
        self.goalUpdate = 0
        self.resume()

    def certSpeed(self, ID, angleInput, speedSet):
        self.scMode = 'certain'
        self.goalUpdate = 1
        for i in range(0, len(ID)):
            newGoal = self.initPos[ID[i]] + \
                self.pwmGenOut(angleInput[i])*self.sc_direction[ID[i]]
            if newGoal > self.maxPos[ID[i]]: newGoal = self.maxPos[ID[i]]
            elif newGoal < self.minPos[ID[i]]: newGoal = self.minPos[ID[i]]
            self.goalPos[ID[i]] = newGoal
        self.speedUpdate(ID, speedSet)
        self.goalUpdate = 0
        self.resume()

    def moveWiggle(self):
        self.bufferPos[self.wiggleID] += self.wiggleDirection*self.sc_direction[self.wiggleID] * \
            self.pwmGenOut(self.scSpeed[self.wiggleID])/(1/self.scDelay)
        newNow = int(round(self.bufferPos[self.wiggleID], 0))
        if self.bufferPos[self.wiggleID] > self.maxPos[self.wiggleID]:
            self.bufferPos[self.wiggleID] = self.maxPos[self.wiggleID]
        elif self.bufferPos[self.wiggleID] < self.minPos[self.wiggleID]: self.bufferPos[self.wiggleID] = self.minPos[self.wiggleID]
        self.nowPos[self.wiggleID] = newNow
        self.lastPos[self.wiggleID] = newNow
        if self.bufferPos[self.wiggleID] < self.maxPos[self.wiggleID] and self.bufferPos[self.wiggleID] > self.minPos[self.wiggleID]:
            pwm.set_pwm(self.wiggleID, 0, self.nowPos[self.wiggleID])
        else:
            self.stopWiggle()
        time.sleep(self.scDelay-self.scMoveTime)

    def stopWiggle(self):
        self.pause()
        self.posUpdate()

    def move(self, part, direction, speed=None):
        id = {"camera": 11, "arm": 12, "hand": 13, "rotate": 14, "grab": 15}[
            part]
        if id in [11, 13]:
            direction = {"up": -1, "down":1}[direction]
        elif id in [12, 14, 15]:
            direction = {"grab": 1, "right": 1, "loose": -1, "left": -1,
                         "up": -1, "down": 1}[direction]
        speed = 2 if speed is None else speed
        self.singleServo(id, direction, speed)

    def singleServo(self, ID, direcInput, speedSet):
        self.wiggleID = ID
        self.wiggleDirection = direcInput
        self.scSpeed[ID] = speedSet
        self.scMode = 'wiggle'
        self.posUpdate()
        self.resume()

    def moveAngle(self, ID, angleInput):
        self.nowPos[ID] = int(
            self.initPos[ID] + self.sc_direction[ID]*self.pwmGenOut(angleInput))
        if self.nowPos[ID] > self.maxPos[ID]: self.nowPos[ID] = self.maxPos[ID]
        elif self.nowPos[ID] < self.minPos[ID]: self.nowPos[ID] = self.minPos[ID]
        self.lastPos[ID] = self.nowPos[ID]
        pwm.set_pwm(ID, 0, self.nowPos[ID])

    def scMove(self):
        if self.scMode == 'init':
            self.moveInit()
        elif self.scMode == 'auto':
            self.moveAuto()
        elif self.scMode == 'certain':
            self.moveCert()
        elif self.scMode == 'wiggle':
            self.moveWiggle()

    def setPWM(self, ID, PWM_input):
        self.lastPos[ID] = PWM_input
        self.nowPos[ID] = PWM_input
        self.bufferPos[ID] = float(PWM_input)
        self.goalPos[ID] = PWM_input
        pwm.set_pwm(ID, 0, PWM_input)
        self.pause()

    def run(self):
        while not self.stop_flag:
            self.__flag.wait()
            # print('debug')
            self.scMove()
            pass

    def stop(self):
        while self.executing:
            time.sleep(1)
        print('stopped')
        self.goalPos = self.initPos
        self.moveAuto()
        self.stop_flag = True
        self.resume()
        # raise SystemExit


if __name__ == '__main__':
    sc = ServoCtrl()
    sc.start()
    # sc.run()
    # print(f'1:{sc.initPos}')
    sc.moveInit()
    sc.goalPos[12] = 560
    sc.moveAuto()
    time.sleep(1)
    sc.goalPos[12] = 400
    sc.moveAuto()
    time.sleep(1)
    sc.stop()
    # while 1:
    #     sc.moveAngle(0,(random.random()*100-50))
    #     time.sleep(1)
    #     sc.moveAngle(1,(random.random()*100-50))
    #     time.sleep(1)
    #     '''
    #     sc.singleServo(0, 1, 5)
    #     time.sleep(6)
    #     sc.singleServo(0, -1, 30)
    #     time.sleep(1)
    #     '''
    #     '''
    #     delaytime = 5
    #     sc.certSpeed([0,7], [60,0], [40,60])
    #     print('xx1xx')
    #     time.sleep(delaytime)

    #     sc.certSpeed([0,7], [0,60], [40,60])
    #     print('xx2xx')
    #     time.sleep(delaytime+2)

    #     # sc.moveServoInit([0])
    #     # time.sleep(delaytime)
    #     '''
    #     '''
    #     pwm.set_pwm(0,0,560)
    #     time.sleep(1)
    #     pwm.set_pwm(0,0,100)
    #     time.sleep(2)
    #     '''
    #     pass
    # pass
