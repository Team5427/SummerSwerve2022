package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.10033;
        public static final double kDriveMotorGearRatio = ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));
        public static final double kTurningMotorGearRatio = ((15.0 / 32.0) * (10.0 / 60.0));
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.6;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(21);   //FIXME
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5);   //FIXME
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 8;   //FIXME
        public static final int kBackLeftDriveMotorPort = 2;   //FIXME
        public static final int kFrontRightDriveMotorPort = 6;   //FIXME
        public static final int kBackRightDriveMotorPort = 4;   //FIXME

        public static final int kFrontLeftTurningMotorPort = 7;   //FIXME
        public static final int kBackLeftTurningMotorPort = 1;   //FIXME
        public static final int kFrontRightTurningMotorPort = 5;   //FIXME
        public static final int kBackRightTurningMotorPort = 3;   //FIXME

        public static final boolean kFrontLeftTurningEncoderReversed = true;   //FIXME
        public static final boolean kBackLeftTurningEncoderReversed = true;   //FIXME
        public static final boolean kFrontRightTurningEncoderReversed = true;   //FIXME
        public static final boolean kBackRightTurningEncoderReversed = true;   //FIXME

        public static final boolean kFrontLeftDriveEncoderReversed = true;   //FIXME
        public static final boolean kBackLeftDriveEncoderReversed = true;   //FIXME
        public static final boolean kFrontRightDriveEncoderReversed = false;   //FIXME
        public static final boolean kBackRightDriveEncoderReversed = false;   //FIXME

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;   //FIXME
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;   //FIXME
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;   //FIXME
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;   //FIXME

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;   //FIXME no idea how to get these vvvvv
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;   //FIXME ???
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;   //FIXME ???
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;   //FIXME ???

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5880.0 / 60.0 * ModuleConstants.kDriveMotorGearRatio * ModuleConstants.kWheelDiameterMeters * Math.PI;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(kTrackWidth / 2, kWheelBase / 2); //12.6 r/s -> 9.22

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }
}
