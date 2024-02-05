import wpilib


class HexEncoder:
    def robotInit(self) -> None:
        """Robot initialization function"""

        self.dutyCycleEncoder = wpilib.DutyCycleEncoder(0)

        self.dutyCycleEncoder.setDistancePerRotation(1)
    
        self.output = 1

    def robotPeriodic(self):
        # Connected can be checked, and uses the frequency of the encoder
        connected = self.dutyCycleEncoder.isConnected()

        # Duty Cycle Frequency in Hz
        frequency = self.dutyCycleEncoder.getFrequency()

        # Output of encoder
        self.output = self.dutyCycleEncoder.get()

        # Output scaled by DistancePerPulse
        distance = self.dutyCycleEncoder.getDistance()