#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import rev
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory
from wpimath.units import inchesToMeters

kWheelRadius = inchesToMeters(2)
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau
# relative to the wheel
kGearRatio = 4/27


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      CAN output for the drive motor.
        :param turningMotorChannel:    CAN output for the turning motor.
        """
        self.driveMotor = rev.CANSparkMax(driveMotorChannel, rev.CANSparkMax.MotorType.kBrushless)
        self.turningMotor = rev.CANSparkMax(turningMotorChannel, rev.CANSparkMax.MotorType.kBrushless)

        self.driveEncoder = self.driveMotor.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        self.turningEncoder = self.turningMotor.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)

        # Configure drive PID Controller
        self.drivePIDController = wpimath.controller.PIDController(1, 0, 0)

        # Configure turning PID Controller+
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            1,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        # Convert rotations to meters for the drive motor.
        self.driveEncoder.setPositionConversionFactor(kGearRatio * kWheelRadius * math.tau)
        self.driveEncoder.setVelocityConversionFactor(kGearRatio * kWheelRadius * math.tau / 60)

        # Convert rotations to radians for the turning motor.
        self.turningEncoder.setPositionConversionFactor(math.tau)
        self.turningEncoder.setVelocityConversionFactor(math.tau / 60)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getPosition()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getPosition()),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        encoderRotation = wpimath.geometry.Rotation2d(self.turningEncoder.getPosition())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getVelocity(), state.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getPosition(), state.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)