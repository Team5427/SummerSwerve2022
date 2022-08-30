package frc.robot.commands.Trajectory;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class AutonController {

    public SwerveDrive m_robotDrive;
    private PratsSwerveAutonCommand finalCommand;
    private Trajectory usedTraj;

    public AutonController(Trajectory traj) {
        this.usedTraj = traj;
        init(usedTraj);
    }

    public AutonController() {
        TrajectoryConfig config = new TrajectoryConfig(
            Constants.MAX_PHYSICAL_SPEED_M_PER_SEC,
            Constants.MAX_AUTON_ACCEL_M_PER_S2)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.SWERVE_DRIVE_KINEMATICS);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                List.of(
                    new Pose2d(new Translation2d(Units.feetToMeters(25.053), Units.feetToMeters(5.604)), new Rotation2d(Units.feetToMeters(0),Units.feetToMeters(-4.137))), 
                    new Pose2d(new Translation2d(Units.feetToMeters(25.866), Units.feetToMeters(2.175)), new Rotation2d(Units.feetToMeters(-0.849), Units.feetToMeters(-1.273))), 
                    new Pose2d(new Translation2d(Units.feetToMeters(23.144), Units.feetToMeters(1.185)), new Rotation2d(Units.feetToMeters(-1.803), Units.feetToMeters(-0.035))), 
                    new Pose2d(new Translation2d(Units.feetToMeters(21.5), Units.feetToMeters(3)), new Rotation2d(Units.feetToMeters(5.675), Units.feetToMeters(10.56)))),
                config);
        init(exampleTrajectory);
    }

    public PratsSwerveAutonCommand getAutonCommand() {
        return finalCommand;
    }

    private void init(Trajectory traj) {
        m_robotDrive = RobotContainer.getSwerve();

        var thetaController = new ProfiledPIDController(
                Constants.AUTON_THETA_P, 0, 0, Constants.THETA_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PratsSwerveAutonCommand pratsSwerveAutonCommandEx = new PratsSwerveAutonCommand(
            traj,
            m_robotDrive::getPose, // Functional interface to feed supplier
            Constants.SWERVE_DRIVE_KINEMATICS,
            new PIDController(Constants.AUTON_X_P, 0, 0),
            new PIDController(Constants.AUTON_Y_P, 0, 0),
            thetaController,
            m_robotDrive::moveSwerve,
            m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(traj.getInitialPose());
        finalCommand = pratsSwerveAutonCommandEx;
    }
}