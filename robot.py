#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#from msilib.schema import Component
from wpilib.cameraserver import CameraServer
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
from components import arm
from components import Shooter
from components import intakemotor
from dataclasses import dataclass, fields
#from components import vision 
from wpilib import SmartDashboard
from rev import CANSparkMax
from networktables import NetworkTables
from components.limelight import LimeLight
from components.drivetrain import Drivetrain


@dataclass
class MotorGroupConfig:
    left_motor: int
    right_motor: int

LEFT_MOTOR_GROUP_CONFIG = MotorGroupConfig(
    left_motor=3, right_motor=4)
RIGHT_MOTOR_GROUP_CONFIG = MotorGroupConfig(
    left_motor=1, right_motor=2)

def create_motor_group(
        motor_group_config: MotorGroupConfig) -> list[CANSparkMax]:
    return [CANSparkMax(getattr(motor_group_config, field.name), CANSparkMax.MotorType.kBrushless) for field in fields(motor_group_config)]

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.joystick = wpilib.Joystick(0)
        self.intake = intakemotor.MotorIntake(8)
        self.shooter = Shooter.ReciprocalMotors(11, 7)
        self.shoulder = arm.ReciprocalMotors(10,9)
        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)
        self.dutyCycle = wpilib.DutyCycle(wpilib.DigitalInput(1))
        self.pdp = wpilib.PowerDistribution()
        NetworkTables.initialize(server="10.67.58.2")
        # self.limelight = NetworkTables.getTable("limelight-kmrobot")
        # print(f"tv: {self.limelight.getEntry("tv")}")

                # hex encoder stuff
        print(self.dutyCycle.getOutput())

        """Robot initialization function"""

        self.dutyCycleEncoder = wpilib.DutyCycleEncoder(0)

        self.dutyCycleEncoder.setDistancePerRotation(1)
        
        self.drive_stick = wpilib.XboxController(1)
        self.robotDrive = Drivetrain()
        self.autonTimer = wpilib.Timer()

        wpilib.CameraServer.launch()
        self.limelight = NetworkTables.getTable("limelight")
        print(self.limelight.getNumber('<tx>', 1))

    def  setMotors(self, forward:float, turn: float):
        self.robotDrive.drive(forward, turn)    

    def autonomousInit(self) -> None:
        self.autonTimer.start()

    def autonomousPeriodic(self) -> None:
        if self.autonTimer.get() <= .5:
            if self.dutyCycle.getOutput() >= 0.361:
                self.shoulder.set(-0.5)
        elif self.autonTimer.get() >= .6:
            self.shooter.set(0.5)
            self.intake.set(1)
        elif self.autonTimer.get() >= .9:
            self.setMotors(0.5, 0.0)
        elif self.autonTimer.get() >= 1.1: 
            self.setMotors(0.0 , 0.0)
        # pass



    def robotPeriodic(self) -> None:
        # voltage = self.pdp.getVoltage()
        # wpilib.SmartDashboard.putNumber("Voltage", voltage)
        # temperatureCelcius = self.pdp.getTemperature()
        # wpilib.SmartDashboard.putNumber("Temperature Celcius", temperatureCelcius)
        # totalPower = self.pdp.getTotalPower()
        # wpilib.SmartDashboard.putNumber("Total Power", totalPower)
        pass
    def teleopInit(self) -> None:
        # self.swerve.resetToAbsolute()
        pass


    def teleopPeriodic(self) -> None:

        print(self.dutyCycle.getOutput())
        CameraServer.is_alive()
        print(CameraServer.is_alive())

        #     # Duty Cycle Frequency in Hz
        # frequency = self.dutyCycleEncoder.getFrequency()
        #     # Output of encoder
        # output = self.dutyCycleEncoder.getAbsolutePosition()
        #     # Output scaled by DistancePerPulse
        # distance = self.dutyCycleEncoder.getDistance()

        # # self.driveWithJoystick(True)
        # # Duty Cycle Frequency in Hz
        # frequency = self.dutyCycle.getFrequency()
        # # Output of duty cycle, ranging from 0 to 1
        # # 1 is fully on, 0 is fully off
        # output = self.dutyCycle.getOutput()

        # wpilib.SmartDashboard.putNumber("Frequency", frequency)
        # wpilib.SmartDashboard.putNumber("Duty Cycle", output)

        wpimath.applyDeadband(self.joystick.getY(), 0.5)

        self.robotDrive.drive(-self.drive_stick.getRawAxis(1), -self.drive_stick.getRawAxis(0))

       
        if self.joystick.getRawButton(2):
            self.intake.set(2)
        else:
            self.intake.set(0)
        #Shooter speed.
        if self.joystick.getRawButton(1):
            # self.drive_stick.setRumble(self.drive_stick.RumbleType.kBothRumble, 0.1)
            self.shooter.set(self.joystick.getRawButton(1) * .55)
        elif self.joystick.getRawButton(3):
            # self.drive_stick.setRumble(self.drive_stick.RumbleType.kBothRumble, 0.1)
            self.shooter.set(-self.joystick.getRawButton(3) * .55)
        else:
            self.shooter.set(0)
            # self.drive_stick.setRumble(self.drive_stick.RumbleType.kBothRumble, 0)
        #End of shooter speed.
        
        self.shoulder.set(self.joystick.getY() * .5)

        

        if self.joystick.getRawButton(6):
            self.shoulder.set(0.3)
            if self.dutyCycle.getOutput() == 0.9750:
                self.shoulder.set(0)
                

        if self.joystick.getRawButton(4):
            self.shoulder.set(0.3)
            if self.dutyCycle.getOutput() == 0.778:
                self.shoulder.set(0)
        # LimeLight.lighting
        # if self.joystick.getRawButton(11):
        #     self.setMotors(0, 1)
        # else:
        #     self.setMotors(0,0)