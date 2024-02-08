import wpilib
import wpilib.drive
from dataclasses import dataclass, fields
#from components import vision 
from rev import CANSparkMax


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
    
class drivetrain(wpilib.TimedRobot):
    def robotInit(self) -> None:
        leftMotor = create_motor_group(LEFT_MOTOR_GROUP_CONFIG)
        rightMotor = create_motor_group(RIGHT_MOTOR_GROUP_CONFIG)
        self.drive_stick = wpilib.XboxController(1)
        self.leftGroup = wpilib.MotorControllerGroup(*leftMotor)
        self.rightGroup = wpilib.MotorControllerGroup(*rightMotor)

        

        self.rightGroup.setInverted(True)
        self.robotDrive = wpilib.drive.DifferentialDrive(self.leftGroup, self.rightGroup)        
        self.robotDrive.setExpiration(0.1)
    def drive(self):
        self.robotDrive.arcadeDrive(-self.drive_stick.getRawAxis(1), -self.drive_stick.getRawAxis(0))