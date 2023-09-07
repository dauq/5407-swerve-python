import sys
from pathlib import Path

import commands2
import wpilib

sys.path.append(str(Path(__file__).parent.parent.absolute()))

from container import RobotContainer


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()
    # def testInit(self) -> None:
    #     self.scheduler.cancelAll
    # def testPeriodic(self) -> None:
    #     wpilib.SmartDashboard.putNumber("Abs Encoder", 1.0)


if __name__ == "__main__":
    wpilib.run(Robot)
