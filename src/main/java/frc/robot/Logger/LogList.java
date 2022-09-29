package frc.robot.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

public class LogList {
    private static SwerveDrive dt;
    private static SwerveModule frontLeft, frontRight, backLeft, backRight;
    public static void fillLogList() {
        dt = RobotContainer.getSwerve();
        frontLeft = dt.getModules()[0];
        frontRight = dt.getModules()[1];
        backLeft = dt.getModules()[2];
        backRight = dt.getModules()[3];
    }

    public static HashMap<String , Object> driveTrainLogList() {
        HashMap<String, Object> map = new HashMap<String, Object>();
        map.putAll(Map.of(
            "Front Left Abs Enc Raw", frontLeft.getAbsEncRaw(),
            "Front Right Abs Enc Raw", frontRight.getAbsEncRaw(),
            "Back Left Abs Enc Raw", backLeft.getAbsEncRaw(),
            "Back Right Abs Enc Raw", backRight.getAbsEncRaw()
        ));
        return map;
    }
}
