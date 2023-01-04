package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AprilTagPi extends SubsystemBase {

    private PhotonCamera cam;
    private PhotonTrackedTarget target;

    public AprilTagPi(String name) {
        // cam = new PhotonCamera(name);
    }

    // @Override
    // public void periodic() {
    //     if (cam.getLatestResult().hasTargets()) {
    //         target = cam.getLatestResult().getBestTarget();
    //     } else {
    //         //no target handler
    //         List<TargetCorner> fakeCorners = List.of(new TargetCorner(0, 0), new TargetCorner(0, 0), new TargetCorner(0, 0), new TargetCorner(0, 0));
    //         target = new PhotonTrackedTarget(0, 0, 0, 0, 0, new Transform3d(new Translation3d(), new Rotation3d()), new Transform3d(new Translation3d(), new Rotation3d()), 0, fakeCorners);
    //     }
    // }

    public PhotonTrackedTarget getTarget() {
        return target;
    }

    public boolean hasTarget() {
        return cam.getLatestResult().hasTargets();
    }

    public Pose3d getTargetFieldPose() {
        if (target.getFiducialId() == 1) {
            return new Pose3d(0, 0, 0, new Rotation3d());
        } else if (target.getFiducialId() == 2) {
            return new Pose3d(0, 0, 0, new Rotation3d());
        } else {
            return new Pose3d();
        }
    }

    public Transform3d getRobotToCamera() {
        return new Transform3d(Constants.CAMERA_TRANSLATION, Constants.CAMERA_ROTATION);
    }

    public Pose2d getVisionBasedRobotPose() {
        return getTargetFieldPose().plus(target.getBestCameraToTarget().inverse()).plus(getRobotToCamera().inverse()).toPose2d();
    }
}