package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private static SwerveSubsystem swerveSubsystem;
    private static Double xSpdFunction, ySpdFunction, turningSpdFunction;
    private static Boolean toggleOrientation;
    private static SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean isFieldOriented;
    ChassisSpeeds chassisSpeeds;

    public SwerveJoystickCmd() {

        addRequirements(RobotContainer.getSwerve());
    }

    @Override
    public void initialize() {        
        swerveSubsystem = RobotContainer.getSwerve();
        xSpdFunction = RobotContainer.getController().getLeftY();
        ySpdFunction = RobotContainer.getController().getLeftX();
        turningSpdFunction = RobotContainer.getController().getRightX();
        toggleOrientation = RobotContainer.getController().getAButton();        
        xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    }

    @Override
    public void execute() {

        double xSpeed = xSpdFunction;
        double ySpeed = ySpdFunction;
        double turningSpeed = turningSpdFunction;

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        if (toggleOrientation) {
            isFieldOriented = !isFieldOriented;
        }

        if (isFieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else if (!isFieldOriented) {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SmartDashboard.putBoolean("Field Oriented", isFieldOriented);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
