// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Commands.PratsSwerveAutonCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.nio.file.FileSystem;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private static final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    private static XboxController m_driverController = new XboxController(0);
    // private static JoystickButton fieldOPBtn = new
    // JoystickButton(m_driverController, 1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.

        // Note: X, Y, and rot for Throttle/Strafe/Rotation is inverted to match WPILib
        // coordinate system
        var driveCommand = new RunCommand(
                () -> m_robotDrive.drive());

        driveCommand.addRequirements(m_robotDrive);

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(driveCommand);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        // fieldOPBtn.whenPressed(() -> m_robotDrive.fieldRelative =
        // !m_robotDrive.fieldRelative);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                List.of(
                    new Pose2d(new Translation2d(Units.feetToMeters(25.053), Units.feetToMeters(5.604)), new Rotation2d(Units.feetToMeters(0),Units.feetToMeters(-4.137))), 
                    new Pose2d(new Translation2d(Units.feetToMeters(25.866), Units.feetToMeters(2.175)), new Rotation2d(Units.feetToMeters(-0.849), Units.feetToMeters(-1.273))), 
                    new Pose2d(new Translation2d(Units.feetToMeters(23.144), Units.feetToMeters(1.185)), new Rotation2d(Units.feetToMeters(-1.803), Units.feetToMeters(-0.035))), 
                    new Pose2d(new Translation2d(Units.feetToMeters(21.5), Units.feetToMeters(3)), new Rotation2d(Units.feetToMeters(5.675), Units.feetToMeters(10.56)))),
                config);


        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PratsSwerveAutonCommand pratsSwerveAutonCommandEx = new PratsSwerveAutonCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        PratsSwerveAutonCommand pratsSwerveAutonCommand = new PratsSwerveAutonCommand(
                Robot.trajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

                

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(Robot.trajectory.getInitialPose());

        // m_robotDrive.showCurrentTrajectory(Robot.trajectory);

        // Run path following command, then stop at the end.
        return pratsSwerveAutonCommand;
    }

    public static XboxController getJoy() {
        return m_driverController;
    }

    public static DriveSubsystem getDrive() {
        return m_robotDrive;
    }

}
