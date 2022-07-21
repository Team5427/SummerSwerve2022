package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

    private CANSparkMax speedMotor;
    private CANSparkMax turnMotor;
    private RelativeEncoder speedEnc;
    private RelativeEncoder turnEnc;
    private CANCoder absEnc;
    private PIDController turningPID;

    public SwerveModule (CANSparkMax speedMotor, CANSparkMax turnMotor, RelativeEncoder speedEnc, RelativeEncoder turnEnc, CANCoder absEnc) {
        this.speedMotor = speedMotor;
        this.turnMotor = turnMotor;
        this.speedEnc = speedEnc;
        this.turnEnc = turnEnc;
        this.absEnc = absEnc;
    }

    public void init() {
        turningPID = new PIDController(Constants.MODULE_KP_CONSTANT, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
        speedEnc.setPositionConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_DEG_TO_METER);
        speedEnc.setVelocityConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S);
        turnEnc.setPositionConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_DEG_TO_RAD);
        turnEnc.setVelocityConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S);
        speedEnc.setPosition(0);
        absEnc.configFactoryDefault();
        absEnc.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnEnc.setPosition(Math.toRadians(absEnc.getAbsolutePosition()));
    }

    public double getDrivePos() {return speedEnc.getPosition();}
    public double getDriveSpeed() {return speedEnc.getVelocity();}
    public double getTurnPosRad() {return turnEnc.getPosition();}
    public double getTurnVel() {return turnEnc.getPosition();}

    public SwerveModuleState getModState() {
        return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getTurnPosRad()));
    }

    public void setModState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
        } else{
            state = SwerveModuleState.optimize(state, getModState().angle);
            speedMotor.set(state.speedMetersPerSecond / Constants.MAX_PHYSICAL_SPEED_M_PER_SEC);
            turnMotor.set(turningPID.calculate(getTurnPosRad(), state.angle.getRadians()));
        }
    }

    public void stop() {
        speedMotor.set(0);
        turnMotor.set(0);
    }
}
