// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

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

  private static CANCoder frontLeftAbsEnc;
  private static CANCoder frontRightAbsEnc;
  private static CANCoder backLeftAbsEnc;
  private static CANCoder backRightAbsEnc;

  private static SwerveModule frontLeft;
  private static SwerveModule frontRight;
  private static SwerveModule backLeft;
  private static SwerveModule backRight;

  private static SwerveDrive swerveDrive;

  private static AHRS ahrs;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //havent started reversals yet
    frontLeftSpeedMotor = new CANSparkMax(0, MotorType.kBrushless);
    frontRightSpeedMotor = new CANSparkMax(0, MotorType.kBrushless);
    backLeftSpeedMotor = new CANSparkMax(0, MotorType.kBrushless);
    backRightSpeedMotor = new CANSparkMax(0, MotorType.kBrushless);
    
    frontLeftTurnMotor = new CANSparkMax(0, MotorType.kBrushless);
    frontRightTurnMotor = new CANSparkMax(0, MotorType.kBrushless);
    backLeftTurnMotor = new CANSparkMax(0, MotorType.kBrushless);
    backRightTurnMotor = new CANSparkMax(0, MotorType.kBrushless);
    
    frontLeftSpeedEncoder = frontLeftSpeedMotor.getEncoder();
    frontRightSpeedEncoder = frontRightSpeedMotor.getEncoder();
    backLeftSpeedEncoder = backLeftSpeedMotor.getEncoder();
    backRightSpeedEncoder = backRightSpeedMotor.getEncoder();

    frontLeftTurnEncoder = frontLeftTurnMotor.getEncoder();
    frontRightTurnEncoder = frontRightTurnMotor.getEncoder();
    backLeftTurnEncoder = backLeftTurnMotor.getEncoder();
    backRightTurnEncoder = backRightTurnMotor.getEncoder();

    frontLeftAbsEnc = new CANCoder(0);
    frontRightAbsEnc = new CANCoder(0);
    backLeftAbsEnc = new CANCoder(0);
    backRightAbsEnc = new CANCoder(0);

    frontLeft = new SwerveModule(frontLeftSpeedMotor, frontLeftTurnMotor, frontLeftSpeedEncoder, frontLeftTurnEncoder, frontLeftAbsEnc);
    frontRight = new SwerveModule(frontRightSpeedMotor, frontRightTurnMotor, frontRightSpeedEncoder, frontRightTurnEncoder, frontRightAbsEnc);
    backLeft = new SwerveModule(backLeftSpeedMotor, backLeftTurnMotor, backLeftSpeedEncoder, backLeftTurnEncoder, backLeftAbsEnc);
    backRight = new SwerveModule(backRightSpeedMotor, backRightTurnMotor, backRightSpeedEncoder, backRightTurnEncoder, backRightAbsEnc);

    frontLeft.init();
    frontRight.init();
    backLeft.init();
    backRight.init();

    ahrs = new AHRS(SPI.Port.kMXP);

    swerveDrive = new SwerveDrive(frontLeft, frontRight, backLeft, backRight, ahrs);

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
