// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    //Robot Ports
    public static final int FRONT_LEFT_SPEED_MOTOR = 8;
    public static final int FRONT_RIGHT_SPEED_MOTOR = 3;
    public static final int BACK_LEFT_SPEED_MOTOR = 6;
    public static final int BACK_RIGHT_SPEED_MOTOR = 2;

    public static final int FRONT_LEFT_TURN_MOTOR = 9;
    public static final int FRONT_RIGHT_TURN_MOTOR = 7;
    public static final int BACK_LEFT_TURN_MOTOR = 5;
    public static final int BACK_RIGHT_TURN_MOTOR = 10;

    public static final int FRONT_LEFT_CANCODER = 12;
    public static final int FRONT_RIGHT_CANCODER = 13;
    public static final int BACK_LEFT_CANCODER = 14;
    public static final int BACK_RIGHT_CANCODER = 11;

    public static final double FRONT_LEFT_OFFSET = 2.25;
    public static final double FRONT_RIGHT_OFFSET = 0.64;
    public static final double BACK_LEFT_OFFSET = 2.97;
    public static final double BACK_RIGHT_OFFSET = 0.754;
    
    //Inversions FIXME all of these
    public static final boolean FRONT_LEFT_TURNING_INVERT = true;
    public static final boolean FRONT_RIGHT_TURNING_INVERT = true;
    public static final boolean BACK_LEFT_TURNING_INVERT = true;
    public static final boolean BACK_RIGHT_TURNING_INVERT = true;

    public static final boolean FRONT_LEFT_DRIVE_INVERT = false;
    public static final boolean FRONT_RIGHT_DRIVE_INVERT = false;
    public static final boolean BACK_LEFT_DRIVE_INVERT = true;
    public static final boolean BACK_RIGHT_DRIVE_INVERT = true;

    public static final boolean FRONT_LEFT_CANCODER_INVERT = true;
    public static final boolean FRONT_RIGHT_CANCODER_INVERT = true;
    public static final boolean BACK_LEFT_CANCODER_INVERT = true;
    public static final boolean BACK_RIGHT_CANCODER_INVERT = true;

    //Robot Physical Dimensions
    public static final double DT_WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double DT_TRACKWIDTH = Units.inchesToMeters(17.5625); //FIXME
    public static final double DT_WHEELBASE = Units.inchesToMeters(19.5); //FIXME

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(DT_WHEELBASE / 2, -DT_TRACKWIDTH / 2),
        new Translation2d(DT_WHEELBASE / 2, DT_TRACKWIDTH / 2),
        new Translation2d(-DT_WHEELBASE / 2, -DT_TRACKWIDTH / 2),
        new Translation2d(-DT_WHEELBASE / 2, DT_TRACKWIDTH / 2));

    //Swerve Speed Numbers
    public static final double kDriveMotorGearRatio = ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
    public static final double kTurningMotorGearRatio = ((15.0 / 32.0) * (10.0 / 60.0));
    public static final double SWERVE_CONVERSION_FACTOR_ROT_TO_METER = (Math.PI * DT_WHEEL_DIAMETER_METERS * kDriveMotorGearRatio);
    public static final double SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S = SWERVE_CONVERSION_FACTOR_ROT_TO_METER / 60;
    public static final double SWERVE_CONVERSION_FACTOR_ROT_TO_RAD = 2 * Math.PI * kTurningMotorGearRatio;
    public static final double SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S = SWERVE_CONVERSION_FACTOR_ROT_TO_RAD / 60;
    public static final double MAX_PHYSICAL_SPEED_M_PER_SEC = 4.4196;
    public static final double CONTROLLER_DEADBAND = 0.15;
    public static final double MAX_ACCEL_TELEOP_PERCENT_PER_S = 3;
    public static final double MAX_ANGULAR_ACCEL_TELEOP_PERCENT_PER_S = 3;
    public static final double MAX_SPEED_TELEOP_M_PER_S = 3;
    public static final double MAX_ANGULAR_SPEED_TELEOP_RAD_PER_S = 2 * Math.PI;
    public static final double MAX_NEO_SPEED_RPM = 5676.0;

    //AUTON STUFF
    public static final double MAX_AUTON_ACCEL_M_PER_S2 = 1;
    public static final double MAX_AUTON_ANGULAR_ACCEL_RAD_PER_S2 = Math.PI; 
    public static final double AUTON_X_P = 0.001; //FIXME
    public static final double AUTON_Y_P = 0.001; //FIXME
    public static final double AUTON_THETA_P = 0.001; //FIXME

    public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_ANGULAR_SPEED_TELEOP_RAD_PER_S, MAX_AUTON_ANGULAR_ACCEL_RAD_PER_S2);

    //CONTROLLER CONSTANTS MODULES
    public static final double TURNING_PID_P = 2.25;
    public static final double TURNING_PID_D = 0.029483;
    public static final double TURNING_FF_S = 0.15325;
    public static final double TURNING_FF_V = 0.019597;
    public static final double TURNING_FF_A = 0.00044001;
    public static final double TURNING_MAX_SPEED_RAD_S = SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S * MAX_NEO_SPEED_RPM;
    public static final double TURNING_MAX_ACCEL_RAD_S_S = SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S * MAX_NEO_SPEED_RPM * 3;

    public static final double SPEED_PID_P = 0.5; //FIXME need to do sysid vvvvvv
    public static final double SPEED_FF_S = 0.5; //FIXME
    public static final double SPEED_FF_V = 0.5; //FIXME
    public static final double SPEED_FF_A = 0.5; //FIXME

    public static enum SwerveModuleType {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }
}
