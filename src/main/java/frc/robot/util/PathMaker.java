package frc.robot.util;

import java.util.HashMap;
import java.util.Set;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class PathMaker {
    private static HashMap<String, PratsSwerveControllerCommand> commandList = new HashMap<String, PratsSwerveControllerCommand>();
    private static SwerveDrive driveTrain;
    private static PIDController xTranslationPID, yTranslationPID;
    private static PIDController thetaPID;

    public static void initPaths(String... sArgs) {
        Set<String> args = Set.of(sArgs);
        xTranslationPID = new PIDController(Constants.AUTON_TRANSLATION_P, 0, 0);
        yTranslationPID = new PIDController(Constants.AUTON_TRANSLATION_P, 0, 0);
        thetaPID = new PIDController(Constants.AUTON_THETA_P, 0, 0);
        driveTrain = RobotContainer.getSwerve();
        args.forEach((c) -> {
            commandList.put(c, new PratsSwerveControllerCommand(
                PathPlanner.loadPath(c, Constants.MAX_AUTON_SPEED_M_PER_S, Constants.MAX_AUTON_ACCEL_M_PER_S2), 
                driveTrain::getPose, 
                Constants.SWERVE_DRIVE_KINEMATICS, 
                xTranslationPID,
                yTranslationPID,
                thetaPID,
                driveTrain::setModules,
                driveTrain)
            );
        });
    }

    public static PratsSwerveControllerCommand getCommand(String name) {
        return commandList.get(name);
    }

    public static void resetPaths() {
        Set<String> s = commandList.keySet();
        commandList.clear();
        initPaths(s.toArray(String[]::new));
    }
}
