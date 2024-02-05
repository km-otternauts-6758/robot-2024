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
from dataclasses import dataclass, fields
from components import drivetrain
from components.swervemodule import SwerveModule
from components import hex
from components import reciprocalmotors
from components import Shooter 
from components import intakemotor
from rev import CANSparkMax

#import booglefub


@dataclass
class MotorGroupConfig:
    left_motor: int
    right_motor: int

LEFT_MOTOR_GROUP_CONFIG = MotorGroupConfig(
    left_motor=3, right_motor=1)
RIGHT_MOTOR_GROUP_CONFIG = MotorGroupConfig(
    left_motor=2, right_motor=4)

def create_motor_group(
        motor_group_config: MotorGroupConfig) -> list[CANSparkMax]:
    return [CANSparkMax(getattr(motor_group_config, field.name), CANSparkMax.MotorType.kBrushless) for field in fields(motor_group_config)]
    
class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.joystick = wpilib.Joystick(0)
        self.drive_stick = wpilib.XboxController(1)        
        #self.swerve = drivetrain.Drivetrain()
        self.intake = intakemotor.IntakeMotors(8)
        self.shooter = Shooter.ReciprocalMotors(11, 7)
        self.shoulder = reciprocalmotors.ReciprocalMotors(10,9)
        self.hexEncoder = hex.HexEncoder()

        # self.drive = tankdrive.DifferentialDrive(tankdrive.robot.robotInit.left_group, tankdrive.robot.robotInit.right_group) 

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        #hex encoder stuff
        #print(self.hexEncoder.output)

        #Shoulder Speed
        self.shoulder.set(0.05)
        
        leftMotor = create_motor_group(LEFT_MOTOR_GROUP_CONFIG)
        rightMotor = create_motor_group(RIGHT_MOTOR_GROUP_CONFIG)

        self.leftGroup = wpilib.MotorControllerGroup(*leftMotor)
        self.rightGroup = wpilib.MotorControllerGroup(*rightMotor)

        

        self.rightGroup.setInverted(True)
        self.robotDrive = wpilib.drive.DifferentialDrive(self.leftGroup, self.rightGroup)        
        self.robotDrive.setExpiration(0.1)
    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(False)
        #self.swerve.updateOdometry()

        #self.intake.set(self.joystick.getY())

        #self.shooter.set(self.joystick.getX())
        
        


    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.joystick.getY(), 0.05)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
            self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.joystick.getX(), 0.05)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            self.rotLimiter.calculate(
                wpimath.applyDeadband(self.joystick.getZ(), 0.05)
            )
            * drivetrain.kMaxSpeed
        )

        #self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        

    def teleopPeriodic(self) -> None:
        #self.driveWithJoystick(True)
        self.intake.set(self.joystick.getRawButton(2))
        if self.joystick.getRawButton(1):
            self.shooter.set(self.joystick.getRawButton(1))
        elif self.joystick.getRawButton(3):
            self.shooter.set(-self.joystick.getRawButton(3))
        else:
            self.shooter.set(0)
        self.shoulder.set(self.joystick.getY())
        #self.drive_stick.setRumble(self.drive_stick.RumbleType.kBothRumble, 1)
        # tankdrive.robot.robotInit.goon(
        #     -self.drive_stick.getLeftY(), self.drive_stick.getRightX()
        #     )
        #self.drive.set(self.drive_stick.getRightX())
        self.robotDrive.arcadeDrive(-self.drive_stick.getRawAxis(1), -self.drive_stick.getRawAxis(0))