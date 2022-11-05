package frc.robot.commands.Auton;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.PathMaker;

public class TestAuton extends SequentialCommandGroup{
    public TestAuton() {
        // addRequirements(RobotContainer.getSwerve());
        String auton1 = "Test1";
        String auton2 = "Test2";

        addCommands(
            new InstantCommand(() -> {
                // RobotContainer.getSwerve().setGyroOffset(PathMaker.getTraj(auton2).getInitialPose().getRotation().getDegrees());
                RobotContainer.getSwerve().resetOdometry(PathMaker.getTraj(auton2).getInitialPose());            
            }),
            PathMaker.getCommand(auton2),
            new InstantCommand(() -> {
                PathMaker.initPaths(auton1, auton2);
                RobotContainer.getSwerve().stopMods();
            })
        );
    }
}
