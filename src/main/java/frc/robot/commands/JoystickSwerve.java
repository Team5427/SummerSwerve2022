package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class JoystickSwerve extends CommandBase {
    
    private SwerveModuleState[] states;
    private XboxController joy;
    private SwerveDrive swerve;

    public JoystickSwerve () {
        addRequirements(RobotContainer.getSwerve());
        joy = RobotContainer.getController();
        swerve = RobotContainer.getSwerve();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (joy.getAButtonPressed()) {swerve.toggleFieldRelative();}
        if (joy.getBButtonPressed()) {
            swerve.zeroHeading();
            swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
            swerve.resetMods();
        }
        states = swerve.controllerToModuleStates(joy);
        swerve.setModules(states);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopMods();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
