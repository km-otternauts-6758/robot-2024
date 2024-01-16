#!/usr/bin/env python3

import wpilib
from rev import CANSparkMax


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.motor_one = CANSparkMax(1, CANSparkMax.MotorType.kBrushless)
        self.motor_two = CANSparkMax(2, CANSparkMax.MotorType.kBrushless)
        self.motor_two.follow(self.motor_one, True)
        self.stick = wpilib.Joystick(0)
        self.timer = wpilib.Timer()

    def autonomousInit(self):
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        pass

    def teleopPeriodic(self):
        self.motor_one.set(self.stick.getY())


if __name__ == "__main__":
    wpilib.run(MyRobot)