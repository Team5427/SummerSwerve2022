// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.JoystickSwerve;
import frc.robot.commands.Trajectory.AutonController;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static AutonController autonController;
  private static SwerveDrive swerveDrive;
  private static AHRS ahrs;
  private static XboxController joy;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    ahrs = new AHRS(SPI.Port.kMXP);
    joy = new XboxController(0);

    swerveDrive = new SwerveDrive(ahrs);
    swerveDrive.setDefaultCommand(new JoystickSwerve());

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joy, XboxController.Button.kB.value).whenPressed(() -> {
      swerveDrive.zeroHeading();
      swerveDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }, swerveDrive);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    autonController = new AutonController();
    return autonController.getAutonCommand().andThen(() -> getSwerve().stopMods());
  }

  public static SwerveDrive getSwerve() {return swerveDrive;}
  public static AHRS getAHRS() {return ahrs;}
  public static XboxController getController() {return joy;}
}
