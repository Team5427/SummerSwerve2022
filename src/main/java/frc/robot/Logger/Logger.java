package frc.robot.Logger;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class Logger extends SubsystemBase {
    public Logger() {
        LogList.fillLogList();
    }

    // @Override
    // public void periodic() {

    // }

    private void setUpShuffleboard() {
    }
}
