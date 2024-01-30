#!/usr/bin/env python3

import wpilib
import rev
import math
from enum import Enum

# 2 inches in meters
kWheelRadius = 0.0508
kGearRatio = 4/27

class MotorState(Enum):
    INIT = 0
    START = 1
    LOG = 2
    DONE = 3

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.timer = wpilib.Timer()
        self.motorState = MotorState.INIT
        
        self.motor = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
        self.motorEncoder = self.motor.getEncoder()
        self.pid = self.motor.getPIDController()
        self.pid.setP(1)
        self.motorEncoder.setPositionConversionFactor(kGearRatio * kWheelRadius * math.tau)
        self.motorEncoder.setVelocityConversionFactor(kGearRatio * kWheelRadius * math.tau / 60)

    def autonomousInit(self):
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        self.motorEncoder.setPosition(0)
        self.timer.reset()
        self.timer.start()
        self.motorState = MotorState.START

    def teleopPeriodic(self):
        if self.motorState == MotorState.INIT:
            raise Exception('self.motorState cannot be in "INIT" state')
        elif self.motorState == MotorState.START:
            if self.motorEncoder.getPosition() < 10:
                self.motor.set(0.5)
            else:
                self.motor.set(0)
                self.motorState = MotorState.LOG

        elif self.motorState == MotorState.LOG:
            print(f"Velocity: {self.motorEncoder.getVelocity()}")
            print(f"Time: {self.timer.get()}")
            print(f"PID: {self.pid.getP()}, {self.pid.getI()}, {self.pid.getD()}")
            self.motorState = MotorState.DONE

        elif self.motorState == MotorState.DONE:
            pass

if __name__ == "__main__":
    wpilib.run(MyRobot)