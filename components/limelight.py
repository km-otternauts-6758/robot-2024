import wpilib
from networktables import NetworkTables

class LimeLight:
    def robotInit(self):    
        NetworkTables.initialize(server="10.67.58.2")
        self.limelight = NetworkTables.getTable("limelight-kmrobot")
        print(f"tv: {self.limelight.getEntry("tv")}")
    def lighting(self):
        if self.limelight.getEntry("ta") > 5.5:
            self.limelight.putNumber("ledMode", 1)
        else:
            self.limelight.putNumber("ledMode", 3)


