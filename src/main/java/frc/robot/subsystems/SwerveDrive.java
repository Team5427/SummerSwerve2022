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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    public SwerveDrive (AHRS m_gyro) {
        this.frontLeft = new SwerveModule(Constants.SwerveModuleType.FRONT_LEFT);
        this.frontRight = new SwerveModule(Constants.SwerveModuleType.FRONT_RIGHT);
        this.backLeft = new SwerveModule(Constants.SwerveModuleType.BACK_LEFT);
        this.backRight = new SwerveModule(Constants.SwerveModuleType.BACK_RIGHT);
        this.gyro = m_gyro;
        isFieldRelative = Constants.FIELD_RELATIVE_ON_START;
        dampener = Constants.DAMPENER_ON_START;
        xRateLimiter = new SlewRateLimiter(Constants.MAX_ACCEL_TELEOP_PERCENT_PER_S);
        yRateLimiter = new SlewRateLimiter(Constants.MAX_ACCEL_TELEOP_PERCENT_PER_S);
        xRateLimiter2 = new SlewRateLimiter(Constants.MAX_ANGULAR_ACCEL_TELEOP_PERCENT_PER_S);
        odometer = new SwerveDriveOdometry(Constants.SWERVE_DRIVE_KINEMATICS, new Rotation2d(0));

        field = new Field2d();
        SmartDashboard.putData(field);

        zeroHeading();
    }

    public void zeroHeading() {
        gyro.zeroYaw();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
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
        xSpeed = controller.getLeftX() * dampener;
        ySpeed = -controller.getLeftY() * dampener;
        x2Speed = Math.pow(controller.getRightX() * dampener, 3);

        xSpeed = Math.abs(xSpeed) > (Constants.CONTROLLER_DEADBAND * dampener) ? xSpeed : 0; //apply deadband
        ySpeed = Math.abs(ySpeed) > (Constants.CONTROLLER_DEADBAND * dampener) ? ySpeed : 0;
        x2Speed = Math.abs(x2Speed) > (Constants.CONTROLLER_DEADBAND * dampener) ? x2Speed : 0;

        xSpeed = xRateLimiter.calculate(xSpeed) * Constants.MAX_SPEED_TELEOP_M_PER_S; //apply slew + scale to m/s and rad/s
        ySpeed = yRateLimiter.calculate(ySpeed) * Constants.MAX_SPEED_TELEOP_M_PER_S;
        x2Speed = xRateLimiter2.calculate(x2Speed) * Constants.MAX_ANGULAR_SPEED_TELEOP_RAD_PER_S;

        chassisSpeeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, x2Speed, getRotation2d()) : new ChassisSpeeds(ySpeed, xSpeed, x2Speed);

        SmartDashboard.putBoolean("FieldOP", isFieldRelative);
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("x2Speed", x2Speed);    
        
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

    public void setDampener(double s) {
        dampener = Constants.DAMPENER_ENABLED ? s : dampener;
    }

    private void log() {
        // SmartDashboard.putNumber("setpoint state abs: front left", frontLeft.getAbsEncRaw());
        // SmartDashboard.putNumber("setpoint state abs: front Right", frontRight.getAbsEncRaw());
        // SmartDashboard.putNumber("setpoint state abs: back left", backLeft.getAbsEncRaw());
        // SmartDashboard.putNumber("setpoint state abs: back Right", backRight.getAbsEncRaw());

        // SmartDashboard.putNumber("setpoint state rel: front left", frontLeft.getTurnPosRad());
        // SmartDashboard.putNumber("setpoint state rel: front Right", frontRight.getTurnPosRad());
        // SmartDashboard.putNumber("setpoint state rel: back left", backLeft.getTurnPosRad());
        // SmartDashboard.putNumber("setpoint state rel: back Right", backRight.getTurnPosRad());

        // // SmartDashboard.putNumber("neo val", frontRight.getTurnPosRad());
        // // SmartDashboard.putNumber("neo val left", frontLeft.getTurnPosRad());
        // SmartDashboard.putNumber("front left speed", frontLeft.getDriveSpeed());
        // SmartDashboard.putNumber("front right speed", frontRight.getDriveSpeed());
        // SmartDashboard.putNumber("back left speed", backLeft.getDriveSpeed());
        // SmartDashboard.putNumber("back right speed", backRight.getDriveSpeed());

        // SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("thing", backRight.getTurnPosRad() % (2 * Math.PI));
        // SmartDashboard.putBoolean("GyroCalibrating", gyro.isCalibrating());
    }

}
