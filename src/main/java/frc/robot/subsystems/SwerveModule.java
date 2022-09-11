package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

    private CANSparkMax speedMotor;
    private CANSparkMax turnMotor;
    private RelativeEncoder speedEnc;
    private RelativeEncoder turnEnc;
    private CANCoder absEnc;
    private ProfiledPIDController turningPID;
    private SimpleMotorFeedforward turningFF;
    private double encoderOffset;

    public SwerveModule (CANSparkMax speedMotor, CANSparkMax turnMotor, RelativeEncoder speedEnc, RelativeEncoder turnEnc, CANCoder absEnc, double encoderOffset) {
        this.speedMotor = speedMotor;
        this.turnMotor = turnMotor;
        this.speedEnc = speedEnc;
        this.turnEnc = turnEnc;
        this.absEnc = absEnc;
        this.encoderOffset = encoderOffset;
    }

    public void init() {
        turningPID = new ProfiledPIDController(Constants.TURNING_PID_P, Constants.TURNING_PID_D, 0, 
            new Constraints(Constants.TURNING_MAX_SPEED_RAD_S, Constants.TURNING_MAX_ACCEL_RAD_S_S));
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
        turningFF = new SimpleMotorFeedforward(Constants.TURNING_FF_S, Constants.TURNING_FF_V, Constants.TURNING_FF_A);
        speedEnc.setPositionConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_ROT_TO_METER);
        speedEnc.setVelocityConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S);
        turnEnc.setPositionConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_ROT_TO_RAD);
        turnEnc.setVelocityConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S);
        speedEnc.setPosition(0);
        absEnc.configFactoryDefault();
        absEnc.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnEnc.setPosition(Math.toRadians(absEnc.getAbsolutePosition()) - encoderOffset);
    }

    public double getDrivePos() {return speedEnc.getPosition();}
    public double getDriveSpeed() {return speedEnc.getVelocity();}
    public double getTurnPosRad() {return turnEnc.getPosition();}
    public double getTurnVel() {return turnEnc.getVelocity();}

    public SwerveModuleState getModState() {
        return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getTurnPosRad()));
    }

    public void setModState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
        } else{
            state = SwerveModuleState.optimize(state, getModState().angle);
            speedMotor.set(state.speedMetersPerSecond / Constants.MAX_PHYSICAL_SPEED_M_PER_SEC);
            turnMotor.setVoltage(turningPID.calculate(getTurnPosRad(), state.angle.getRadians()) + turningFF.calculate(turningPID.getSetpoint().velocity));
        }
    }

    public void stop() {
        speedMotor.set(0);
        turnMotor.set(0);
    }
}
