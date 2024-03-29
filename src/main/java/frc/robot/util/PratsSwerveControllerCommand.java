package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Custom PathPlanner version of SwerveControllerCommand
 */
public class PratsSwerveControllerCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final PathPlannerTrajectory trajectory;
    private final Supplier<Pose2d> poseSupplier;
    private final SwerveDriveKinematics kinematics;
    private final PPHolonomicDriveController controller;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final HashMap<String, Command> eventMap;
    private final Field2d field = new Field2d();
    private final Runnable stopMods;

    private ArrayList<PathPlannerTrajectory.EventMarker> unpassedMarkers;

    /**
     * Constructs a new PratsSwerveControllerCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory         The trajectory to follow.
     * @param poseSupplier       A function that supplies the robot pose - use one of the odometry classes to provide this.
     * @param kinematics         The kinematics for the robot drivetrain.
     * @param xController        The Trajectory Tracker PID controller for the robot's x position.
     * @param yController        The Trajectory Tracker PID controller for the robot's y position.
     * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
     * @param outputModuleStates The raw output module states from the position controllers.
     * @param stopModules        Stops modules after the command finishes.
     * @param eventMap           Map of event marker names to the commands that should run when reaching that marker.
     *                           This SHOULD NOT contain any commands requiring the same subsystems as this command, or it will be interrupted
     * @param requirements       The subsystems to require.
     */
    public PratsSwerveControllerCommand(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            PIDController rotationController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Runnable stopModules,
            HashMap<String, Command> eventMap,
            Subsystem... requirements) {
        this.trajectory = trajectory;
        this.poseSupplier = poseSupplier;
        this.kinematics = kinematics;
        this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
        this.outputModuleStates = outputModuleStates;
        this.eventMap = eventMap;
        this.stopMods = stopModules;

        addRequirements(requirements);
    }


    /**
     * Constructs a new PratsSwerveControllerCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory         The trajectory to follow.
     * @param poseSupplier       A function that supplies the robot pose - use one of the odometry classes to provide this.
     * @param kinematics         The kinematics for the robot drivetrain.
     * @param xController        The Trajectory Tracker PID controller for the robot's x position.
     * @param yController        The Trajectory Tracker PID controller for the robot's y position.
     * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
     * @param outputModuleStates The raw output module states from the position controllers.
     * @param stopModules        Stops modules after the command finishes.
     * @param requirements       The subsystems to require.
     */
    public PratsSwerveControllerCommand(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            PIDController rotationController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Runnable stopModules,
            Subsystem... requirements) {
        this(trajectory, poseSupplier, kinematics, xController, yController, rotationController, outputModuleStates, stopModules, new HashMap<>(), requirements);
    }

    @Override
    public void initialize() {
        this.unpassedMarkers = new ArrayList<>();
        this.unpassedMarkers.addAll(this.trajectory.getMarkers());

        Logger.Work.postComplex("PratsSwerveControllerCommand_field", this.field, BuiltInWidgets.kField); //might not work
        Logger.Work.postComplex("PratsSwerveControllerCommand_field", this.field, BuiltInWidgets.kField);
        this.field.getObject("traj").setTrajectory(this.trajectory);

        this.timer.reset();
        this.timer.start();

        PathPlannerServer.sendActivePath(this.trajectory.getStates());
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        PathPlannerState desiredState = (PathPlannerState) this.trajectory.sample(currentTime);

        Pose2d currentPose = this.poseSupplier.get();
        this.field.setRobotPose(currentPose);
        PathPlannerServer.sendPathFollowingData(new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation), currentPose);

        Logger.Work.post("PratsSwerveControllerCommand_xError", currentPose.getX() - desiredState.poseMeters.getX());
        Logger.Work.post("PratsSwerveControllerCommand_yError", currentPose.getY() - desiredState.poseMeters.getY());
        Logger.Work.post("PratsSwerveControllerCommand_rotationError", currentPose.getRotation().getRadians() - desiredState.holonomicRotation.getRadians());

        ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);
        SwerveModuleState[] targetModuleStates = this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

        this.outputModuleStates.accept(targetModuleStates);

        if(this.unpassedMarkers.size() > 0 && currentTime >= this.unpassedMarkers.get(0).timeSeconds) {
            PathPlannerTrajectory.EventMarker marker = this.unpassedMarkers.remove(0);

            if(this.eventMap.containsKey(marker.name)) {
                Command command = this.eventMap.get(marker.name);

                command.schedule();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();
        if (this.trajectory.getEndState().velocityMetersPerSecond <= .05)
            this.stopMods.run(); //watches out for non-zero endstates

        if(interrupted){
            this.outputModuleStates.accept(this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
        }
    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds());
    }

    public PathPlannerTrajectory getTrajectory() {
        return this.trajectory;
    }
}

