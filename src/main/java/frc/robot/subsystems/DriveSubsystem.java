// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.stream.Collectors;

import static frc.robot.Constants.DriveConstants.*;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
    // Robot swerve modules

    public boolean fieldRelative;
    public boolean trackTarget;
    public boolean dynamicTarget;
    private double xSpeed;
    private double ySpeed;
    private double rot;
    private double finalTargetAngle;
    private boolean isKB = false;
    private double getAxisX;
    private double getAxisY;
    private double getAxisTheta;
    private SlewRateLimiter limiterX;
    private SlewRateLimiter limiterY;
    private SlewRateLimiter limiterZ;

    private final SwerveModule m_frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderPorts,
            DriveConstants.kFrontLeftTurningEncoderPorts,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed);

    private final SwerveModule m_rearLeft = new SwerveModule(
            DriveConstants.kRearLeftDriveMotorPort,
            DriveConstants.kRearLeftTurningMotorPort,
            DriveConstants.kRearLeftDriveEncoderPorts,
            DriveConstants.kRearLeftTurningEncoderPorts,
            DriveConstants.kRearLeftDriveEncoderReversed,
            DriveConstants.kRearLeftTurningEncoderReversed);

    private final SwerveModule m_frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderPorts,
            DriveConstants.kFrontRightTurningEncoderPorts,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed);

    private final SwerveModule m_rearRight = new SwerveModule(
            DriveConstants.kRearRightDriveMotorPort,
            DriveConstants.kRearRightTurningMotorPort,
            DriveConstants.kRearRightDriveEncoderPorts,
            DriveConstants.kRearRightTurningEncoderPorts,
            DriveConstants.kRearRightDriveEncoderReversed,
            DriveConstants.kRearRightTurningEncoderReversed);

    private final SwerveModule[] m_swerveModules = {
            m_frontLeft,
            m_frontRight,
            m_rearLeft,
            m_rearRight
    };

    // The gyro sensor
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    private final ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(m_gyro);

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

    Pose2d[] m_modulePose = {
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
    };

    Field2d m_fieldSim = new Field2d();

    // Simulation variables
    private double m_yawValue;
    private double raw_angle;
    private double raw_angle_sign;
    private double curr_angle;
    private double target_x, target_y, robot_x, robot_y, target_x_mid, target_y_mid;
    private PIDController targettrackPID;
    private double turningCounter;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

        SmartDashboard.putData("Field", m_fieldSim);
        fieldRelative = true;
        trackTarget = false;
        dynamicTarget = false;
        turningCounter = 0;
        // xSpeed = RobotContainer.getJoy().getRawAxis(0) * DriveConstants.kMaxSpeedMetersPerSecond;
        // ySpeed = -RobotContainer.getJoy().getRawAxis(1) * DriveConstants.kMaxSpeedMetersPerSecond;
        // rot = -RobotContainer.getJoy().getRawAxis(2) * DriveConstants.kMaxChassisAngularSpeedRadiansPerSecond;
        targettrackPID = new PIDController(0.4, 0, 0);
        targettrackPID.enableContinuousInput(1, 360);
        resetOdometry(new Pose2d(8.811, 6.263, new Rotation2d(0)));
        limiterX = new SlewRateLimiter(4);
        limiterY = new SlewRateLimiter(4);
        limiterZ = new SlewRateLimiter(8);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                new Rotation2d(getHeading()),
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());

        // Update the poses for the swerveModules. Note that the order of rotating the
        // position and then
        // adding the translation matters
        for (int i = 0; i < m_swerveModules.length; i++) {
            var modulePositionFromChassis = kModulePositions[i]
                    .rotateBy(new Rotation2d(getHeading()))
                    .plus(getPose().getTranslation());

            // Module's heading is it's angle relative to the chassis heading
            m_modulePose[i] = new Pose2d(modulePositionFromChassis,
                    m_swerveModules[i].getState().angle.plus(getPose().getRotation()));
        }

        m_fieldSim.setRobotPose(getPose());
        // m_fieldSim.getObject("Swerve Modules").setPoses(m_modulePose);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
        resetEncoders();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive() {

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(getHeading()))
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);

        SmartDashboard.putNumber("gyro deg", getGyroDeg(true));
        // SmartDashboard.putString("coords", getPose().getTranslation().toString());
        // SmartDashboard.putNumber("target_angle_raw", getTargetAngle());
        // SmartDashboard.putNumber("target_angle_dynamic", getDynamicTargetAngle());
        // SmartDashboard.putNumber("quadrant", getQuadrant());
        // SmartDashboard.putNumber("x speed", xSpeed);
        // SmartDashboard.putNumber("y speed", ySpeed);
        // SmartDashboard.putNumber("chassis speed", getChassisVectorSpeed());
        // SmartDashboard.putNumber("chassis angle", getChassisVectorAngle());
        SmartDashboard.putNumber("par speed", getChassisParSpeed());
        SmartDashboard.putNumber("targ_x", target_x);
        SmartDashboard.putNumber("targ_y", target_y);
        // SmartDashboard.putNumber("targ_new_angle_x",
        // moveTargetX(getChassisTanSpeed()));
        // SmartDashboard.putNumber("targ_new_angle_y",
        // moveTargetY(getChassisTanSpeed()));

    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearLeft.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in radians(ik its scuffed), from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    public double getGyroDeg(boolean isDeg) {
        raw_angle = Math.abs(-Units.radiansToDegrees(getHeading())) % 360;
        raw_angle_sign = Math.signum(-Units.radiansToDegrees(getHeading()));
        curr_angle = (raw_angle_sign > 0) ? (raw_angle) : (360 - raw_angle);

        if (isDeg) {
            return curr_angle;
        } else if (!isDeg) {
            return Units.degreesToRadians(curr_angle);
        } else {
            return 0.0;
        }
    }

    public double getTargetAngle() {
        double x_dist = Math.abs(target_x_mid - robot_x);
        double y_dist = Math.abs(target_y_mid - robot_y);
        double raw_target_angle = Units.radiansToDegrees(Math.atan(y_dist / x_dist));

        if (getQuadrant() == 1) {
            finalTargetAngle = -raw_target_angle + 360;
        } else if (getQuadrant() == 2) {
            finalTargetAngle = raw_target_angle;
        } else if (getQuadrant() == 3) {
            finalTargetAngle = 180 - raw_target_angle;
        } else if (getQuadrant() == 4) {
            finalTargetAngle = -(180 - raw_target_angle) + 360;
        }

        return finalTargetAngle;
    }

    public double getDynamicTargetAngle() {
        double x_dist = Math.abs(target_x - robot_x);
        double y_dist = Math.abs(target_y - robot_y);
        double raw_target_angle = Units.radiansToDegrees(Math.atan(y_dist / x_dist));

        if (getDynamicQuadrant() == 1) {
            finalTargetAngle = -raw_target_angle + 360;
        } else if (getDynamicQuadrant() == 2) {
            finalTargetAngle = raw_target_angle;
        } else if (getDynamicQuadrant() == 3) {
            finalTargetAngle = 180 - raw_target_angle;
        } else if (getDynamicQuadrant() == 4) {
            finalTargetAngle = -(180 - raw_target_angle) + 360;
        }

        return finalTargetAngle;
    }

    public double getTargetDist() {
        double x_dist = Math.abs(target_x_mid - robot_x);
        double y_dist = Math.abs(target_y_mid - robot_y);

        return Math.hypot(x_dist, y_dist);
    }

    public int getQuadrant() {
        if (robot_x <= 8.23 && robot_y <= (8.23 / 2)) {
            return 1;
        } else if (robot_x <= 8.23 && robot_y > (8.23 / 2)) {
            return 2;
        } else if (robot_x > 8.23 && robot_y > (8.23 / 2)) {
            return 3;
        } else if (robot_x > 8.23 && robot_y <= (8.23 / 2)) {
            return 4;
        } else {
            return 0;
        }
    }

    public int getDynamicQuadrant() {
        if (robot_x <= target_x && robot_y <= (target_y)) {
            return 1;
        } else if (robot_x <= target_x && robot_y > (target_y)) {
            return 2;
        } else if (robot_x > target_x && robot_y > (target_y)) {
            return 3;
        } else if (robot_x > target_x && robot_y <= (target_y)) {
            return 4;
        } else {
            return 0;
        }
    }

    public int getDirectionQuadrant(double x, double y) {
        if (x <= 0 && y <= 0) {
            return 1;
        } else if (x <= 0 && y > 0) {
            return 2;
        } else if (x > 0 && y > 0) {
            return 3;
        } else if (x > 0 && y <= 0) {
            return 4;
        } else {
            return 0;
        }
    }

    public double getChassisVectorAngle() {
        if (fieldRelative) {
            double raw_angle = Units.radiansToDegrees(Math.atan(Math.abs(ySpeed) / Math.abs(xSpeed)));

            if (getDirectionQuadrant(xSpeed, ySpeed) == 1) {
                raw_angle *= -1;
                raw_angle += 180;
            } else if (getDirectionQuadrant(xSpeed, ySpeed) == 2) {
                raw_angle += 180;
            } else if (getDirectionQuadrant(xSpeed, ySpeed) == 3) {
                raw_angle *= -1;
                raw_angle += 360;
            }

            return raw_angle;
        }

        Rotation2d robotAngle = new Rotation2d(getHeading());

        double xFieldSpeed = -xSpeed * robotAngle.getCos() + ySpeed * robotAngle.getSin();
        double yFieldSpeed = xSpeed * robotAngle.getSin() + ySpeed * robotAngle.getCos();

        xFieldSpeed *= -1;

        double raw_angle = Units.radiansToDegrees(Math.atan(Math.abs(yFieldSpeed) / Math.abs(xFieldSpeed)));

        if (getDirectionQuadrant(xFieldSpeed, yFieldSpeed) == 1) {
            raw_angle *= -1;
            raw_angle += 180;
        } else if (getDirectionQuadrant(xFieldSpeed, yFieldSpeed) == 2) {
            raw_angle += 180;
        } else if (getDirectionQuadrant(xFieldSpeed, yFieldSpeed) == 3) {
            raw_angle *= -1;
            raw_angle += 360;
        }

        return raw_angle;
    }

    public double getChassisVectorSpeed() {
        if (fieldRelative) {
            double speed = Math.hypot(xSpeed, ySpeed);

            if (speed >= 3.657)
                speed = 3.657;

            return speed;
        }

        Rotation2d robotAngle = new Rotation2d(getHeading());
        double xFieldSpeed = xSpeed * robotAngle.getCos() + ySpeed * robotAngle.getSin();
        double yFieldSpeed = -xSpeed * robotAngle.getSin() + ySpeed * robotAngle.getCos();
        double speed = Math.hypot(xFieldSpeed, yFieldSpeed);

        if (speed >= 3.657)
            speed = 3.657;

        return speed;
    }

    public double getChassisTanSpeed() {

        Rotation2d robotAngle = new Rotation2d(getHeading());

        if (fieldRelative) {
            double tanSpeedDesaturated = -xSpeed * robotAngle.getSin() + ySpeed * robotAngle.getCos();
            if (tanSpeedDesaturated > DriveConstants.kMaxSpeedMetersPerSecond) {
                tanSpeedDesaturated = 3.657;
            } else if (tanSpeedDesaturated < -DriveConstants.kMaxSpeedMetersPerSecond) {
                tanSpeedDesaturated = -3.657;
            }
            return tanSpeedDesaturated;
        } else {
            return ySpeed;
        }
    }

    public double getChassisParSpeed() {

        Rotation2d robotAngle = new Rotation2d(getHeading());

        if (fieldRelative) {
            double parSpeedDesaturated = xSpeed * robotAngle.getCos() + ySpeed * robotAngle.getSin();
            if (parSpeedDesaturated > DriveConstants.kMaxSpeedMetersPerSecond) {
                parSpeedDesaturated = 3.657;
            } else if (parSpeedDesaturated < -DriveConstants.kMaxSpeedMetersPerSecond) {
                parSpeedDesaturated = -3.657;
            }
            return parSpeedDesaturated;
        } else {
            return xSpeed;
        }
    }

    public double dynamicVectorOffsetX(double tanV) {
        double kDynamicTargetCoefficientX = 0.33333;
        return tanV * kDynamicTargetCoefficientX;
    }

    public double dynamicVectorOffsetY(double parV) {
        double kDynamicTargetCoefficientY = 0.33333;
        return parV * kDynamicTargetCoefficientY;
    }

    public double moveTargetY(double vectorX, double vectorY) {
        double gyroAngle = getGyroDeg(true);

        double y_val = (Math.cos(Units.degreesToRadians(gyroAngle))) * vectorX
                + Math.sin(Units.degreesToRadians(gyroAngle)) * vectorY;

        return y_val;
    }

    public double moveTargetX(double vectorX, double vectorY) {
        double gyroAngle = getGyroDeg(true);

        double x_val = (Math.sin(Units.degreesToRadians(gyroAngle))) * vectorX
                + Math.cos(Units.degreesToRadians(gyroAngle)) * vectorY;

        return x_val;
    }

    public double moveTargetXFromVector(double speed, double angle) {
        return speed * Math.cos(Units.degreesToRadians(angle)) * DriveConstants.kXTargetTrackCoefficient;
    }

    public double moveTargetYFromVector(double speed, double angle) {
        return speed * Math.sin(Units.degreesToRadians(angle)) * DriveConstants.kYTargetTrackCoefficient;
    }

    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void showCurrentTrajectory(Trajectory trajectory) {
        var trajectoryStates = new ArrayList<Pose2d>();

        trajectoryStates.addAll(trajectory.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));

        m_fieldSim.getObject("Trajectory").setPoses(trajectoryStates);
    }

    @Override
    public void simulationPeriodic() {
        m_frontLeft.simulationPeriodic(0.02);
        m_frontRight.simulationPeriodic(0.02);
        m_rearLeft.simulationPeriodic(0.02);
        m_rearRight.simulationPeriodic(0.02);

        SwerveModuleState[] moduleStates = {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
        };
        if (isKB) {
            getAxisX = limiterX.calculate(SmartDashboard.getNumber("x", 0));
            getAxisY = limiterY.calculate(SmartDashboard.getNumber("y", 0));
            getAxisTheta = limiterZ.calculate(SmartDashboard.getNumber("z", 0));
        } else if (!isKB) {
            getAxisX = RobotContainer.getJoy().getRawAxis(0);
            getAxisY = RobotContainer.getJoy().getRawAxis(1);
            getAxisTheta = RobotContainer.getJoy().getRawAxis(4);
        }

        SmartDashboard.putNumber("limitX", limiterX.calculate(SmartDashboard.getNumber("x", 0)));
        SmartDashboard.putNumber("limitY", limiterY.calculate(SmartDashboard.getNumber("y", 0)));
        SmartDashboard.putNumber("limitZ", limiterZ.calculate(SmartDashboard.getNumber("z", 0)));

        var chassisSpeed = kDriveKinematics.toChassisSpeeds(moduleStates);
        double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

        m_yawValue += chassisRotationSpeed * 0.02 * 0.02;
        m_gyroSim.setAngle(-Units.radiansToDegrees(m_yawValue));

        if (RobotContainer.getJoy().getAButtonPressed())
            fieldRelative = !fieldRelative;

        if (RobotContainer.getJoy().getBButtonPressed())
            trackTarget = !trackTarget;

        // trackTarget = SmartDashboard.getBoolean("targetLock", false);

        if (RobotContainer.getJoy().getXButtonPressed()) {
            dynamicTarget = !dynamicTarget;
        }

        if (fieldRelative) {
            xSpeed = Math.abs(getAxisX) >= 0.15
                    ? getAxisX * DriveConstants.kMaxSpeedMetersPerSecond
                    : 0.0;
            ySpeed = Math.abs(getAxisY) >= 0.15
                    ? -getAxisY * DriveConstants.kMaxSpeedMetersPerSecond
                    : 0.0;

            // VVVVV this irl speeds because daniel is looking from shortside not longside
            // xSpeed = Math.abs(RobotContainer.getJoy().getRawAxis(1)) >= 0.1 ?
            // RobotContainer.getJoy().getRawAxis(1) *
            // DriveConstants.kMaxSpeedMetersPerSecond : 0.0;
            // ySpeed = Math.abs(RobotContainer.getJoy().getRawAxis(0)) >= 0.1 ?
            // RobotContainer.getJoy().getRawAxis(0) *
            // DriveConstants.kMaxSpeedMetersPerSecond : 0.0;

            // rot = -RobotContainer.getJoy().getRawAxis(2) *
            // DriveConstants.kMaxChassisAngularSpeedRadiansPerSecond;

        } else if (!fieldRelative) {
            xSpeed = Math.abs(-getAxisY) >= 0.15
                    ? -getAxisY * DriveConstants.kMaxSpeedMetersPerSecond
                    : 0.0;
            ySpeed = Math.abs(-getAxisX) >= 0.15
                    ? -getAxisX * DriveConstants.kMaxSpeedMetersPerSecond
                    : 0.0;
        }

        if (!trackTarget) {
            rot = Math.abs(-getAxisTheta) >= 0.15
                    ? -getAxisTheta * DriveConstants.kMaxChassisAngularSpeedRadiansPerSecond
                    : 0.0;
        } else if (trackTarget) {
            if (dynamicTarget) {
                rot = -targettrackPID.calculate(getGyroDeg(true), getDynamicTargetAngle());
            } else if (!dynamicTarget) {
                rot = -targettrackPID.calculate(getGyroDeg(true), getTargetAngle());
            }

        }

        if (trackTarget) {
            if (dynamicTarget) {
                m_fieldSim.getObject("target").setPose(target_x, target_y, new Rotation2d(Units.degreesToRadians(turningCounter)));
            } else if (!dynamicTarget) {
                m_fieldSim.getObject("target").setPose(target_x_mid, target_y_mid, new Rotation2d(Units.degreesToRadians(turningCounter)));
            }
        } else if (!trackTarget) {
            m_fieldSim.getObject("target").setPose(8.23, 8.23 / 2, new Rotation2d(Units.degreesToRadians(turningCounter)));
        }

        if (xSpeed != 0 || ySpeed != 0) {
            target_x = 8.23 - moveTargetXFromVector(getChassisVectorSpeed(), getChassisVectorAngle());
            target_y = 8.23 / 2 + moveTargetYFromVector(getChassisVectorSpeed(), getChassisVectorAngle());
        } else {
            target_x = 8.23;
            target_y = 8.23 / 2;
        }

        target_x_mid = 8.23;
        target_y_mid = 8.23 / 2;

        turningCounter += 2; //literally just keeps target turning in place, looks cool

        robot_x = getPose().getX();
        robot_y = getPose().getY();

    }
}
