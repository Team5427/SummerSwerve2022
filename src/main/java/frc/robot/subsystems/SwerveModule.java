package frc.robot.subsystems;

import javax.naming.CannotProceedException;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{

    private static CANSparkMax speedMotor;
    private static CANSparkMax turnMotor;
    private static RelativeEncoder speedEnc;
    private static RelativeEncoder turnEnc;
    private static CANCoder absEnc;
    private static PIDController turningPID;

    public SwerveModule (CANSparkMax speedMotor, CANSparkMax turnMotor, RelativeEncoder speedEnc, RelativeEncoder turnEnc, CANCoder absEnc) {
        this.speedMotor = speedMotor;
        this.turnMotor = turnMotor;
        this.speedEnc = speedEnc;
        this.turnEnc = turnEnc;
        this.absEnc = absEnc;
    }

    public void initModule() {
        turningPID = new PIDController(Constants.MODULE_KP_CONSTANT, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
        speedEnc.setPositionConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_DEG_TO_METER);
        speedEnc.setVelocityConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_RPM_TO_METER_PER_S);
        turnEnc.setPositionConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_DEG_TO_RAD);
        turnEnc.setVelocityConversionFactor(Constants.SWERVE_CONVERSION_FACTOR_RPM_TO_RAD_PER_S);
        
    }
}
