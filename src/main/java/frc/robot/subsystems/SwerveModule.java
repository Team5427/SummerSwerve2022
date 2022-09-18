package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

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
    private Constants.SwerveModuleType m_type;

    public SwerveModule (Constants.SwerveModuleType type) {
        determineIDs(type);
        this.speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        this.speedMotor.setInverted(speedInv);
        this.turnMotor.setInverted(turnInv);
        this.speedEnc = speedMotor.getEncoder();
        this.turnEnc = turnMotor.getEncoder();
        this.absEnc = new CANCoder(absEncID);
        this.m_type = type;
        init();
    }

    public void init() {
        this.speedMotor.setIdleMode(IdleMode.kCoast);
        this.turnMotor.setIdleMode(IdleMode.kCoast);
        this.turningPID = new ProfiledPIDController(Constants.TURNING_PID_P, Constants.TURNING_PID_D, 0, 
            new Constraints(Constants.TURNING_MAX_SPEED_RAD_S, Constants.TURNING_MAX_ACCEL_RAD_S_S));
        this.turningPID.enableContinuousInput(-Math.PI, Math.PI);
        this.turningFF = new SimpleMotorFeedforward(Constants.TURNING_FF_S, Constants.TURNING_FF_V, Constants.TURNING_FF_A);
        this.speedPID = new PIDController(Constants.SPEED_PID_P, 0, 0);
        this.speedFF = new SimpleMotorFeedforward(Constants.SPEED_FF_S, Constants.SPEED_FF_V, Constants.SPEED_FF_A);
        this.speedEnc.setPositionConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_ROT_TO_METER);
        this.speedEnc.setVelocityConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S);
        this.turnEnc.setPositionConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_ROT_TO_RAD);
        this.turnEnc.setVelocityConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S);
        this.speedEnc.setPosition(0);
        this.absEnc.configFactoryDefault();
        this.absEnc.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        this.turnEnc.setPosition(getAbsEncRad());
        // setModState(new SwerveModuleState(0, new Rotation2d(0)));
    }


    public double getDrivePos() {return this.speedEnc.getPosition();}
    public double getDriveSpeed() {return this.speedEnc.getVelocity();}
    public double getTurnPosRad() {return this.turnEnc.getPosition();}
    public double getTurnVel() {return this.turnEnc.getVelocity();}
    public double getAbsEncRaw() {return Math.toRadians(this.absEnc.getAbsolutePosition());}
    public double getAbsEncRad() {
        double x = getAbsEncRaw();
        x -= encoderOffset;
        x = encInv ? (x * -1) + 2* Math.PI: x;
        return x;
    }


    public SwerveModuleState getModState() {
        return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getTurnPosRad()));
    }

    public void setModState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) <= 0.025) {
            stop();
        } else {
            state = SwerveModuleState.optimize(state, getModState().angle);
            // speedMotor.setVoltage(speedPID.calculate(getDriveSpeed(), state.speedMetersPerSecond) + speedFF.calculate(state.speedMetersPerSecond));
            this.speedMotor.set(state.speedMetersPerSecond/Constants.MAX_PHYSICAL_SPEED_M_PER_SEC);
            this.turnMotor.setVoltage(turningPID.calculate(getTurnPosRad(), state.angle.getRadians()) + turningFF.calculate(turningPID.getSetpoint().velocity));
        }
    }

    public void stop() {
        this.speedMotor.set(0);
        this.turnMotor.set(0);
    }

    private void determineIDs(Constants.SwerveModuleType type) {
        if (type.equals(Constants.SwerveModuleType.FRONT_LEFT)) {
            this.speedMotorID = Constants.FRONT_LEFT_SPEED_MOTOR;
            this.turnMotorID = Constants.FRONT_LEFT_TURN_MOTOR;
            this.absEncID = Constants.FRONT_LEFT_CANCODER;
            this.encoderOffset = Constants.FRONT_LEFT_OFFSET;
            this.speedInv = Constants.FRONT_LEFT_DRIVE_INVERT;
            this.turnInv = Constants.FRONT_LEFT_TURNING_INVERT;
            this.encInv = Constants.FRONT_LEFT_CANCODER_INVERT;
        } else if (type.equals(Constants.SwerveModuleType.FRONT_RIGHT)) {
            this.speedMotorID = Constants.FRONT_RIGHT_SPEED_MOTOR;
            this.turnMotorID = Constants.FRONT_RIGHT_TURN_MOTOR;
            this.absEncID = Constants.FRONT_RIGHT_CANCODER;
            this.encoderOffset = Constants.FRONT_RIGHT_OFFSET;
            this.speedInv = Constants.FRONT_RIGHT_DRIVE_INVERT;
            this.turnInv = Constants.FRONT_RIGHT_TURNING_INVERT;
            this.encInv = Constants.FRONT_RIGHT_CANCODER_INVERT;
        } else if (type.equals(Constants.SwerveModuleType.BACK_LEFT)) {
            this.speedMotorID = Constants.BACK_LEFT_SPEED_MOTOR;
            this.turnMotorID = Constants.BACK_LEFT_TURN_MOTOR;
            this.absEncID = Constants.BACK_LEFT_CANCODER;
            this.encoderOffset = Constants.BACK_LEFT_OFFSET;
            this.speedInv = Constants.BACK_LEFT_DRIVE_INVERT;
            this.turnInv = Constants.BACK_LEFT_TURNING_INVERT;
            this.encInv = Constants.BACK_LEFT_CANCODER_INVERT;
        } else if (type.equals(Constants.SwerveModuleType.BACK_RIGHT)) {
            this.speedMotorID = Constants.BACK_RIGHT_SPEED_MOTOR;
            this.turnMotorID = Constants.BACK_RIGHT_TURN_MOTOR;
            this.absEncID = Constants.BACK_RIGHT_CANCODER;
            this.encoderOffset = Constants.BACK_RIGHT_OFFSET;
            this.speedInv = Constants.BACK_RIGHT_DRIVE_INVERT;
            this.turnInv = Constants.BACK_RIGHT_TURNING_INVERT;
            this.encInv = Constants.BACK_RIGHT_CANCODER_INVERT;
        }
    }
}
