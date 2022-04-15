package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class JoystickSwerve extends CommandBase {
    
    SwerveModuleState[] states;
    XboxController joy;
    SwerveDrive swerve;

    public JoystickSwerve () {
        addRequirements(RobotContainer.getSwerve());
    }

    @Override
    public void initialize() {
        joy = RobotContainer.getController();
        swerve = RobotContainer.getSwerve();

    }

    @Override
    public void execute() {
        states = swerve.controllerToModuleStates(joy);
        swerve.moveSwerve(states);
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
