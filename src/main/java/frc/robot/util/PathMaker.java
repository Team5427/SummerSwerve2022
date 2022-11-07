package frc.robot.util;

import java.util.HashMap;
import java.util.Set;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class PathMaker {
    private static HashMap<String, PathPlannerTrajectory> trajList = new HashMap<String, PathPlannerTrajectory>();
    private static HashMap<String, PPSwerveControllerCommand> commandList = new HashMap<String, PPSwerveControllerCommand>();
    private static SwerveDrive driveTrain;
    private static PIDController xTranslationPID, yTranslationPID;
    private static PIDController thetaPID;

    public static void initPaths(Set<String> args) {
        xTranslationPID = new PIDController(Constants.AUTON_TRANSLATION_P, 0, 0);
        yTranslationPID = new PIDController(Constants.AUTON_TRANSLATION_P, 0, 0);
        thetaPID = new PIDController(Constants.AUTON_THETA_P, 0, 0);
        driveTrain = RobotContainer.getSwerve();
        args.forEach((c) -> {
            trajList.put(c, PathPlanner.loadPath(c, Constants.MAX_AUTON_SPEED_M_PER_S, Constants.MAX_AUTON_ACCEL_M_PER_S2));
            commandList.put(c, new PPSwerveControllerCommand(
                trajList.get(c), 
                driveTrain::getPose, 
                Constants.SWERVE_DRIVE_KINEMATICS, 
                xTranslationPID,
                yTranslationPID,
                thetaPID,
                driveTrain::setModules,
                driveTrain)
            ).andThen(() -> {
                resetPaths();
                driveTrain.stopMods();
            });
        });
    }

    public static PPSwerveControllerCommand getCommand(String name) {
        return commandList.get(name);
    }

    public static PathPlannerTrajectory getTraj(String name) {
        return trajList.get(name);
    }

    public static void resetPaths() {
        Set<String> s = commandList.keySet();
        trajList.clear();
        commandList.clear();
        initPaths(s);
    }
}
