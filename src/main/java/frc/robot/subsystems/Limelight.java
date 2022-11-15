package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    private NetworkTable table_m;
    private boolean tv;
    private double tx;
    private double ty;
    private double ledMode;

    public Limelight(NetworkTable table) {
        this.table_m = table;
    }

    @Override
    public void periodic() {
        tv = table_m.getEntry("tv").getDouble(0) == 1;
        tx = table_m.getEntry("tx").getDouble(0);
        ty = table_m.getEntry("ty").getDouble(0);
        ledMode = table_m.getEntry("ledMode").getDouble(0);
        if (DriverStation.isEnabled() || DriverStation.isAutonomous()) {
            setLight(true);
        } else if (DriverStation.isDisabled() || DriverStation.isEStopped()) {
            setLight(false);
        }
    }

    public boolean targetVisible() {
        return tv;
    }

    public double targetX() {
        return tx;
    }

    public double targetY() {
        return ty;
    }

    public boolean lightOn() {
        return ledMode == 0;
    }
    
    public void setLight(boolean on) {
        if (on) {
            table_m.getEntry("ledMode").setNumber(0);
        } else {
            table_m.getEntry("ledMode").setNumber(1);
        }
    }
}
