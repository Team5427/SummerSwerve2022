package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.PathMaker;
import frc.robot.util.PratsSwerveControllerCommand;

public class AutonSheet {
    public static SequentialCommandGroup testAuton;
    private static PratsSwerveControllerCommand test1;
    public static void initAutons() {
        test1 = PathMaker.getCommand("Test1");

        testAuton = new SequentialCommandGroup(
            new InstantCommand(() -> {
                RobotContainer.getSwerve().resetOdometry(test1.getTrajectory().getInitialPose());        
            }),
            test1,
            new InstantCommand(() -> {
                RobotContainer.getSwerve().stopMods();
            })
        );
    }
}
