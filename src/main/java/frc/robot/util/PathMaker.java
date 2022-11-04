package frc.robot.util;

import java.util.HashMap;
import java.util.Set;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class PathMaker {
    private static HashMap<String, PathPlannerTrajectory> trajList = new HashMap<String, PathPlannerTrajectory>();
    private static HashMap<String, PPSwerveControllerCommand> commandList = new HashMap<String, PPSwerveControllerCommand>();
    private static SwerveDrive driveTrain;
    private static PIDController xTranslationPID, yTranslationPID;
    private static PIDController thetaPID;

    public static void initPaths() {
        String[] args = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").toFile().list();
        xTranslationPID = new PIDController(Constants.AUTON_TRANSLATION_P, 0, 0);
        yTranslationPID = new PIDController(Constants.AUTON_TRANSLATION_P, 0, 0);
        thetaPID = new PIDController(Constants.AUTON_THETA_P, 0, 0);
        driveTrain = RobotContainer.getSwerve();
        for (int i = 0; i < args.length; i++) {
            String pathName = args[i].split(".")[0]; //remove .path extension
            trajList.put(pathName, PathPlanner.loadPath(pathName, Constants.MAX_SPEED_TELEOP_M_PER_S, Constants.MAX_AUTON_ACCEL_M_PER_S2));
            commandList.put(pathName, new PPSwerveControllerCommand(
                trajList.get(pathName), 
                driveTrain::getPose, 
                Constants.SWERVE_DRIVE_KINEMATICS, 
                xTranslationPID,
                yTranslationPID,
                thetaPID,
                driveTrain::setModules,
                driveTrain)
            );
        }
    }

    public static PPSwerveControllerCommand getCommand(String name) {
        return commandList.get(name);
    }

    public static PathPlannerTrajectory getTraj(String name) {
        return trajList.get(name);
    }

    public static Set<String> allPathNames() {
        return trajList.keySet();
    }
}
