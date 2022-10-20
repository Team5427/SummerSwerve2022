package frc.robot.util;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveModule {

    private int speedMotorID;
    private int turnMotorID;
    private int absEncID;
    private boolean speedInv;
    private boolean turnInv;
    private boolean encInv;
    private CANSparkMax speedMotor;
    private CANSparkMax turnMotor;
    private RelativeEncoder speedEnc;
    private RelativeEncoder turnEnc;
    private CANCoder absEnc;
    private ProfiledPIDController turningPID;
    private SimpleMotorFeedforward turningFF;
    private PIDController speedPID;
    private SimpleMotorFeedforward speedFF;
    private double encoderOffset;
    private Timer timer;

    public SwerveModule (Constants.SwerveModuleType type) {
        determineIDs(type);
        init();
    }

    public CANSparkMax getDriveSpark() {return speedMotor;}
    public CANSparkMax getTurnSpark() {return turnMotor;}
    public PIDController getDrivePID() {return speedPID;}
    public SimpleMotorFeedforward getDriveFF() {return speedFF;}
    public ProfiledPIDController getTurnPID() {return turningPID;}
    public SimpleMotorFeedforward getTurnFF() {return turningFF;}
    public CANCoder getAbsEnc() {return absEnc;}
    public double getDrivePos() {return speedEnc.getPosition();}
    public double getDriveSpeed() {return speedEnc.getVelocity();}
    public double getTurnPosRad() {return turnEnc.getPosition();}
    public double getTurnVel() {return turnEnc.getVelocity();}
    public double getAbsEncRaw() {return Math.toRadians(absEnc.getAbsolutePosition());}
    public double getAbsEncRad() {
        double x = getAbsEncRaw();
        x -= encoderOffset;
        x = encInv ? (x * -1): x;
        return x;
    }


    public SwerveModuleState getModState() {
        return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getTurnPosRad()));
    }

    public void setModState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) <= 0.02) {
            stop();
        } else {
            state = SwerveModuleState.optimize(fixModule(state), getModState().angle);
            speedMotor.setVoltage(speedPID.calculate(getDriveSpeed(), state.speedMetersPerSecond) + speedFF.calculate(state.speedMetersPerSecond));
            turnMotor.setVoltage(turningPID.calculate(getTurnPosRad(), state.angle.getRadians()) + turningFF.calculate(turningPID.getSetpoint().velocity));
        }
    }

    public void stop() {
        speedMotor.stopMotor();
        turnMotor.stopMotor();
    }

    public SwerveModuleState fixModule(SwerveModuleState p_state) {
        // if (Math.abs(Math.IEEEremainder(getTurnPosRad(), Math.PI) - getAbsEncRad()) > Constants.MODULE_FUCKED_THRESHOLD) {
        if (RobotContainer.getController().getXButton()) {
            timer.start();
            if (timer.get() > 0.25) { //FIXME needs to be tuned
                turnEnc.setPosition(getAbsEncRad());
                timer.stop();
                timer.reset();
            }
            return new SwerveModuleState(p_state.speedMetersPerSecond, new Rotation2d(0));
        } else {
            timer.stop();
            timer.reset();
            return p_state;
        }
    }

    private void determineIDs(Constants.SwerveModuleType type) {
        switch(type) {
            case FRONT_LEFT:
                speedMotorID = Constants.FRONT_LEFT_SPEED_MOTOR;
                turnMotorID = Constants.FRONT_LEFT_TURN_MOTOR;
                absEncID = Constants.FRONT_LEFT_CANCODER;
                encoderOffset = Constants.FRONT_LEFT_OFFSET;
                speedInv = Constants.FRONT_LEFT_DRIVE_INVERT;
                turnInv = Constants.FRONT_LEFT_TURNING_INVERT;
                encInv = Constants.FRONT_LEFT_CANCODER_INVERT;
                break;
            case FRONT_RIGHT:
                speedMotorID = Constants.FRONT_RIGHT_SPEED_MOTOR;
                turnMotorID = Constants.FRONT_RIGHT_TURN_MOTOR;
                absEncID = Constants.FRONT_RIGHT_CANCODER;
                encoderOffset = Constants.FRONT_RIGHT_OFFSET;
                speedInv = Constants.FRONT_RIGHT_DRIVE_INVERT;
                turnInv = Constants.FRONT_RIGHT_TURNING_INVERT;
                encInv = Constants.FRONT_RIGHT_CANCODER_INVERT;
                break;
            case BACK_LEFT:
                speedMotorID = Constants.BACK_LEFT_SPEED_MOTOR;
                turnMotorID = Constants.BACK_LEFT_TURN_MOTOR;
                absEncID = Constants.BACK_LEFT_CANCODER;
                encoderOffset = Constants.BACK_LEFT_OFFSET;
                speedInv = Constants.BACK_LEFT_DRIVE_INVERT;
                turnInv = Constants.BACK_LEFT_TURNING_INVERT;
                encInv = Constants.BACK_LEFT_CANCODER_INVERT;
                break;
            case BACK_RIGHT:
                speedMotorID = Constants.BACK_RIGHT_SPEED_MOTOR;
                turnMotorID = Constants.BACK_RIGHT_TURN_MOTOR;
                absEncID = Constants.BACK_RIGHT_CANCODER;
                encoderOffset = Constants.BACK_RIGHT_OFFSET;
                speedInv = Constants.BACK_RIGHT_DRIVE_INVERT;
                turnInv = Constants.BACK_RIGHT_TURNING_INVERT;
                encInv = Constants.BACK_RIGHT_CANCODER_INVERT;
                break;
        }
    }
    
    public void init() {
        speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        speedMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();
        speedMotor.setSmartCurrentLimit(40);
        turnMotor.setSmartCurrentLimit(27);
        speedMotor.setInverted(speedInv);
        turnMotor.setInverted(turnInv);
        speedEnc = speedMotor.getEncoder();
        turnEnc = turnMotor.getEncoder();
        absEnc = new CANCoder(absEncID);
        speedMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kBrake);
        turningPID = new ProfiledPIDController(Constants.TURNING_PID_P, Constants.TURNING_PID_D, 0, 
            new Constraints(Constants.TURNING_MAX_SPEED_RAD_S, Constants.TURNING_MAX_ACCEL_RAD_S_S));
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
        turningFF = new SimpleMotorFeedforward(Constants.TURNING_FF_S, Constants.TURNING_FF_V, Constants.TURNING_FF_A);
        speedPID = new PIDController(Constants.SPEED_PID_P, 0, 0);
        speedFF = new SimpleMotorFeedforward(Constants.SPEED_FF_S, Constants.SPEED_FF_V, Constants.SPEED_FF_A);
        speedEnc.setPositionConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_ROT_TO_METER);
        speedEnc.setVelocityConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S);
        turnEnc.setPositionConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_ROT_TO_RAD);
        turnEnc.setVelocityConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S);
        speedEnc.setPosition(0);
        absEnc.configFactoryDefault();
        absEnc.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turnEnc.setPosition(getAbsEncRad());
        timer = new Timer();
    }
}
