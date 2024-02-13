#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#from msilib.schema import Component
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
from components import drivetrain

from dataclasses import dataclass, fields
from components import drivetrain
from components.swervemodule import SwerveModule

from components import reciprocalmotors
from components import shooter
#from components import vision 
from rev import CANSparkMax
from wpilib import SmartDashboard
class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.joystick = wpilib.Joystick(0)
        self.swerve = drivetrain.Drivetrain()
        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)
        self.dutyCycle = wpilib.DutyCycle(wpilib.DigitalInput(1))

                # hex encoder stuff
        print(self.dutyCycle.getOutput())

        """Robot initialization function"""

        self.dutyCycleEncoder = wpilib.DutyCycleEncoder(0)

        self.dutyCycleEncoder.setDistancePerRotation(1)
        


    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(False)
        self.swerve.updateOdometry()


    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.joystick.getY(), 0.5)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
            self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.joystick.getX(), 0.5)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            self.rotLimiter.calculate(
                wpimath.applyDeadband(self.joystick.getZ(), 0.5)
            )
            * drivetrain.kMaxSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        

    def teleopInit(self) -> None:
        # self.swerve.resetToAbsolute()
        pass


    def teleopPeriodic(self) -> None:
            # Connected can be checked, and uses the frequency of the encoder
        connected = self.dutyCycleEncoder.isConnected()

            # Duty Cycle Frequency in Hz
        frequency = self.dutyCycleEncoder.getFrequency()

            # Output of encoder
        output = self.dutyCycleEncoder.getAbsolutePosition()

            # Output scaled by DistancePerPulse
        distance = self.dutyCycleEncoder.getDistance()

        # wpilib.SmartDashboard.putBoolean("Connected", connected)
        # wpilib.SmartDashboard.putNumber("Frequency", frequency)
        # wpilib.SmartDashboard.putNumber("Output", output)
        # wpilib.SmartDashboard.putNumber("Distance", distance)

        self.driveWithJoystick(True)
        # Duty Cycle Frequency in Hz
        frequency = self.dutyCycle.getFrequency()

        # Output of duty cycle, ranging from 0 to 1
        # 1 is fully on, 0 is fully off
        # output = self.dutyCycle.getOutput()

        # wpilib.SmartDashboard.putNumber("Frequency", frequency)
        # wpilib.SmartDashboard.putNumber("Duty Cycle", output)