package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class OdometryMath2022 extends SubsystemBase {

    private static Translation2d hubTrans;
    private static Pose2d robotPose;
    private static Translation2d hubTriangleTrans;
    private static Rotation2d hubTriangleRot;

    @Override
    public void periodic() {
        hubTrans = new Translation2d(Units.feetToMeters(27), Units.feetToMeters(27/2));
        robotPose = RobotContainer.getSwerve().getPose();
        hubTriangleTrans = robotPose.getTranslation().minus(hubTrans);
        hubTriangleRot = new Rotation2d(smartArcAngle(hubTriangleTrans.getX(), hubTriangleTrans.getY(), Math.hypot(hubTriangleTrans.getX(), hubTriangleTrans.getY())));
        
    }

    private double smartArcAngle(double inputX, double inputY, double distance) {
        if (inputX < 0 ) {
            return (Math.PI - Math.asin(inputY/distance));
        } else {
            return Math.asin(inputY/distance);
        }
    }

    public static int robotEasiestTurnToTarget() {
        if (hubTriangleRot.minus(robotPose.getRotation()).getRadians() > 0) {
            return 1;
        } else {
            return -1;
        }
    }
}
