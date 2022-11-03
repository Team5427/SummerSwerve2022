package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.PathMaker;

public class TestAuton extends SequentialCommandGroup{
    public TestAuton() {
        addRequirements(RobotContainer.getSwerve());
        addCommands(PathMaker.getCommand("Test1"), new RunCommand(() -> {
            System.out.println("yay it finished");
        }));
    }

    @Override
    public void initialize() {
        RobotContainer.getSwerve().resetOdometry(PathMaker.getTraj("Test1").getInitialPose());
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.getSwerve().stopMods();
    }
}
