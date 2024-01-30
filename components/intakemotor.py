import rev

class IntakeMotors:
    def __init__(self, leftMotorChannel: int) -> None:
        self.leftMotor = rev.CANSparkMax(leftMotorChannel, rev.CANSparkMax.MotorType.kBrushless)

    def set(self, speed: float) -> None:
        self.leftMotor.set(speed)