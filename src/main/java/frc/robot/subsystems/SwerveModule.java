package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase{

    private static CANSparkMax speedMotor;
    private static CANSparkMax turnMotor;
    private static RelativeEncoder speedEnc;
    private static RelativeEncoder turnEnc;
    private static AnalogInput absEnc;

    public SwerveModule (CANSparkMax speedMotor, CANSparkMax turnMotor, RelativeEncoder speedEnc, RelativeEncoder turnEnc, AnalogInput absEnc) {
        this.speedMotor = speedMotor;
        this.turnMotor = turnMotor;
        this.speedEnc = speedEnc;
        this.turnEnc = turnEnc;
        this.absEnc = absEnc;
    }
}
