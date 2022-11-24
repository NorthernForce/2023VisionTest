package frc.robot.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import static frc.robot.RobotContainer.drivetrain;

public class Target {
    private Pose3d coordinates;
    public Target(PhotonTrackedTarget target)
    {
        Transform3d relativeTargetTransform = target.getBestCameraToTarget();
        Transform3d actualTargetPose = relativeTargetTransform.plus(
            new Transform3d(new Pose3d(), drivetrain.getPose3d())
        );
        coordinates = new Pose3d(actualTargetPose.getTranslation(), actualTargetPose.getRotation());
    }
    public void update(PhotonTrackedTarget target)
    {
        Transform3d relativeTargetTransform = target.getBestCameraToTarget();
        Transform3d actualTargetPose = relativeTargetTransform.plus(
            new Transform3d(new Pose3d(), drivetrain.getPose3d())
        );
        coordinates = new Pose3d(actualTargetPose.getTranslation(), actualTargetPose.getRotation());
    }
    public Pose3d getPose3d()
    {
        return coordinates;
    }
}