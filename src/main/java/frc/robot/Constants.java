// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final double DT_WHEEL_DIAMETER_INCHES = 1.5;
    //SwerveStuff
    public static final double SWERVE_CONVERSION_FACTOR_DEG_TO_METER = (Math.PI * DT_WHEEL_DIAMETER_INCHES) / 180;
    public static final double SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S = 6 * SWERVE_CONVERSION_FACTOR_DEG_TO_METER;
    public static final double SWERVE_CONVERSION_FACTOR_DEG_TO_RAD = Math.PI/180;
    public static final double SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S = 6 * SWERVE_CONVERSION_FACTOR_DEG_TO_RAD;
    public static final double MODULE_KP_CONSTANT = 0.0001;
}
