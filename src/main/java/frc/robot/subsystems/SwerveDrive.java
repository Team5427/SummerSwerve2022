package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Logger;
import frc.robot.util.SwerveModule;

public class SwerveDrive extends SubsystemBase {

    private SwerveModule frontLeft, frontRight, backLeft, backRight;
    private AHRS gyro;
    private double xSpeed;
    private double ySpeed;
    private double x2Speed;
    private SlewRateLimiter xRateLimiter, yRateLimiter, xRateLimiter2;
    private ChassisSpeeds chassisSpeeds;
    private boolean isFieldRelative;
    private double dampener;
    private SwerveDriveOdometry odometer;
    private Field2d field;
    private static int resetCounter;

    public SwerveDrive (AHRS m_gyro) {
        this.frontLeft = new SwerveModule(Constants.SwerveModuleType.FRONT_LEFT);
        this.frontRight = new SwerveModule(Constants.SwerveModuleType.FRONT_RIGHT);
        this.backLeft = new SwerveModule(Constants.SwerveModuleType.BACK_LEFT);
        this.backRight = new SwerveModule(Constants.SwerveModuleType.BACK_RIGHT);
        this.gyro = m_gyro;
        isFieldRelative = Constants.FIELD_RELATIVE_ON_START;
        dampener = 1;
        xRateLimiter = new SlewRateLimiter(Constants.MAX_ACCEL_TELEOP_PERCENT_PER_S);
        yRateLimiter = new SlewRateLimiter(Constants.MAX_ACCEL_TELEOP_PERCENT_PER_S);
        xRateLimiter2 = new SlewRateLimiter(Constants.MAX_ANGULAR_ACCEL_TELEOP_PERCENT_PER_S);
        odometer = new SwerveDriveOdometry(Constants.SWERVE_DRIVE_KINEMATICS, new Rotation2d(0));

        field = new Field2d();
        zeroHeading();

        log();
    }

    public void zeroHeading() {
        gyro.zeroYaw();
    }

    public double getHeading() {
        return Math.IEEEremainder((360 - gyro.getAngle()), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopMods() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public SwerveModuleState[] controllerToModuleStates(XboxController controller) {
        dampener = ((Constants.DAMPENER_LOW_PERCENT - 1) * controller.getRightTriggerAxis() + 1);

        xSpeed = -controller.getLeftX() * dampener;
        ySpeed = -controller.getLeftY() * dampener;
        x2Speed = Math.signum(-controller.getRightX()) * Math.pow(Math.abs(controller.getRightX()), Constants.CONTROLLER_TURNING_EXPONENT * dampener) * dampener;
        //dampens exponent as well as speed

        xSpeed = Math.abs(xSpeed) > (Constants.CONTROLLER_DEADBAND * dampener) ? xSpeed : 0; //apply deadband with dampener
        ySpeed = Math.abs(ySpeed) > (Constants.CONTROLLER_DEADBAND * dampener) ? ySpeed : 0;
        x2Speed = Math.abs(x2Speed) > (Constants.CONTROLLER_DEADBAND * dampener) ? x2Speed : 0;

        xSpeed = xRateLimiter.calculate(xSpeed) * Constants.MAX_SPEED_TELEOP_M_PER_S; //apply slew + scale to m/s and rad/s
        ySpeed = yRateLimiter.calculate(ySpeed) * Constants.MAX_SPEED_TELEOP_M_PER_S;
        x2Speed = xRateLimiter2.calculate(x2Speed) * Constants.MAX_ANGULAR_SPEED_TELEOP_RAD_PER_S;

        if (controller.getPOV() != -1) {
            ySpeed = Math.cos(Math.toRadians(360 - controller.getPOV()));
            xSpeed = Math.sin(Math.toRadians(360 - controller.getPOV()));
        }

        chassisSpeeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, x2Speed, getRotation2d()) : new ChassisSpeeds(ySpeed, xSpeed, x2Speed);
        
        //IF YOU ARE WONDERING WHY YSPEED IS IN XSPEED PARAM OF CHASSIS SPEEDS STOP WHAT YOU ARE DOING AND ASK PRAT.
        //DO NOT FLIP.
        //WILL BREAK SPACE TIME FABRIC.
        //DUCK WILL NOT BE PROUD.

        //DO NOT CHANGE ANYTHING HERE
        //EVER
        //THIS IS SACRED CODE
        //IT WAS WRITTEN 35000 FEET IN THE AIR TRAVELING AT 582 MILES PER HOUR

        SwerveModuleState[] states = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        return states;
    }

    public void setModules(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_PHYSICAL_SPEED_M_PER_SEC);
        frontLeft.setModState(desiredStates[0]);
        frontRight.setModState(desiredStates[1]);
        backLeft.setModState(desiredStates[2]);
        backRight.setModState(desiredStates[3]);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    public void resetMods() {
        SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
        for (int i = 0; i < 4; i++) {
            modules[i].getTurnSpark().getEncoder().setPosition(modules[i].getAbsEncRad());
        }
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getModState(), frontRight.getModState(), backLeft.getModState(), backRight.getModState());
        field.setRobotPose(odometer.getPoseMeters());
        
        log();
    }

    public SwerveModule[] getModules() {
        SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
        return modules;
    }

    public boolean getFieldRelative() {
        return isFieldRelative;
    }

    public void toggleFieldRelative() {
        isFieldRelative = Constants.FIELD_RELATIVE_SWITCHABLE ? !isFieldRelative : isFieldRelative;
    }

    public static void incrementResetCounter() {
        resetCounter++;
    }

    private void log() {
        Logger.Work.post("FieldRelative", getFieldRelative());
        // Logger.Work.post("GyroCalibrating", gyro.isCalibrating());
        Logger.Work.post("odom", odometer.getPoseMeters().toString());
        // Logger.Work.post("key", backLeft.getTurnPosRad());
        Logger.Work.post("gyro", getHeading());

        Logger.Work.post("backLeft", Math.IEEEremainder(Math.abs(Math.IEEEremainder(backLeft.getTurnPosRad(), Math.PI) - backLeft.getAbsEncRad()), Math.PI));
        Logger.Work.post("frontLeft", Math.IEEEremainder(Math.abs(Math.IEEEremainder(frontLeft.getTurnPosRad(), Math.PI) - frontLeft.getAbsEncRad()), Math.PI));
        Logger.Work.post("backRight", Math.IEEEremainder(Math.abs(Math.IEEEremainder(backRight.getTurnPosRad(), Math.PI) - backRight.getAbsEncRad()), Math.PI));
        Logger.Work.post("frontRight", Math.IEEEremainder(Math.abs(Math.IEEEremainder(frontRight.getTurnPosRad(), Math.PI) - frontRight.getAbsEncRad()), Math.PI));
        Logger.Work.postComplex("Field542", field, BuiltInWidgets.kField);
        Logger.Work.post("resetCOutner", resetCounter);
        Logger.Work.post("abs FR", frontRight.getAbsEncRad());
        Logger.Work.post("abs FL", frontLeft.getAbsEncRad());
        Logger.Work.post("abs BR", backRight.getAbsEncRad());
        Logger.Work.post("abs BL", backLeft.getAbsEncRad());
    }
}
