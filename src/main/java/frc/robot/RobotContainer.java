package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private static SwerveSubsystem swerveSubsystem;
    private static XboxController driverJoytick;
    private static TrajectorySettings trajSet;

    public RobotContainer() {

        trajSet = new TrajectorySettings(false);
        swerveSubsystem = new SwerveSubsystem();
        driverJoytick = new XboxController(0);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd());

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    }

    public Command getAutonomousCommand() {

        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajSet.trajectory.getInitialPose())),
                TrajectorySettings.getSwerveAuton(),
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    public static SwerveSubsystem getSwerve() {return swerveSubsystem;}
    public static XboxController getController() {return driverJoytick;}

}
