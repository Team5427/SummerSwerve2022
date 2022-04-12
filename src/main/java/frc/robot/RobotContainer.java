// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static CANSparkMax frontLeftSpeedMotor;
  private static CANSparkMax frontRightSpeedMotor;
  private static CANSparkMax backLeftSpeedMotor;
  private static CANSparkMax backRightSpeedMotor;

  private static CANSparkMax frontLeftTurnMotor;
  private static CANSparkMax frontRightTurnMotor;
  private static CANSparkMax backLeftTurnMotor;
  private static CANSparkMax backRightTurnMotor;

  private static RelativeEncoder frontLeftSpeedEncoder;
  private static RelativeEncoder frontRightSpeedEncoder;
  private static RelativeEncoder backLeftSpeedEncoder;
  private static RelativeEncoder backRightSpeedEncoder;

  private static RelativeEncoder frontLeftTurnEncoder;
  private static RelativeEncoder frontRightTurnEncoder;
  private static RelativeEncoder backLeftTurnEncoder;
  private static RelativeEncoder backRightTurnEncoder;

  private static AnalogInput frontLeftAbsEnc;
  private static AnalogInput frontRightAbsEnc;
  private static AnalogInput backLeftAbsEnc;
  private static AnalogInput backRightAbsEnc;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    return null;
  }
}
