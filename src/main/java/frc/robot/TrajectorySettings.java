package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.*;

public class TrajectorySettings {

	private boolean isPathweaver;
	private TrajectoryConfig trajectoryConfig;
	public Trajectory trajectory;
	private String PathJson;
    private Path trajectoryPath;
	private static PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
	private static PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
	private static ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    private static SwerveControllerCommand swerveControllerCommand;

	public TrajectorySettings(boolean isPathweaver) {

		this.isPathweaver = isPathweaver;

        trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

        PathJson = "paths/TestPath1.wpilib.json";

        if (!isPathweaver) {
            trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), 
                List.of(new Translation2d(1, 0), 
                new Translation2d(1, -1)), 
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);
        } else if (isPathweaver) {
            try {
                trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(PathJson);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
             } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + PathJson, ex.getStackTrace());
             }
        }

        swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            RobotContainer.getSwerve()::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            RobotContainer.getSwerve()::setModuleStates,
            RobotContainer.getSwerve());

    }

	public static SwerveControllerCommand getSwerveAuton() {
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		return swerveControllerCommand;
	}

}
