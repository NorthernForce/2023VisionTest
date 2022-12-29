// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.cameraMount;
import static frc.robot.RobotContainer.drivetrain;

public class TrackingSystem extends SubsystemBase {
  public enum CameraFilter
  {
    YELLOW_BALL,
    APRILTAG;
    public int getPipelineIndex()
    {
      switch (this)
      {
      case YELLOW_BALL:
        return 0;
      case APRILTAG:
        return 1;
      default:
        return -1;
      }
    }
  }
  private final PhotonCamera camera;
  private PhotonPipelineResult lastResult;
  private CameraFilter currentFilter;
  private final DifferentialDrivePoseEstimator robotPoseEstimator;
  /** Creates a new TrackingSystem. */
  public TrackingSystem(String cameraName, CameraFilter filter) {
    camera = new PhotonCamera(cameraName);
    camera.setPipelineIndex((currentFilter = filter).getPipelineIndex());
    robotPoseEstimator = new DifferentialDrivePoseEstimator(drivetrain.getHeading(),
      new Pose2d(),
      new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));
  }
  void setFilter(CameraFilter filter)
  {
    camera.setPipelineIndex((currentFilter = filter).getPipelineIndex());
  }
  @Override
  public void periodic() {
    lastResult = camera.getLatestResult();
    robotPoseEstimator.update(drivetrain.getHeading(),
      drivetrain.getWheelSpeeds(), drivetrain.getEncoderRotations()[0], getTargetPitchDegrees());
    if (hasTargets() && currentFilter == CameraFilter.APRILTAG)
    {
      double latency = Timer.getFPGATimestamp() - lastResult.getLatencyMillis();
      Transform2d transform = new Transform2d(new Translation2d(getTransformToTarget().getX(),
        getTransformToTarget().getZ()), new Rotation2d(getTransformToTarget().getRotation().getZ()));
      Pose2d currentPose = Constants.targetPose.transformBy(transform.inverse());
      robotPoseEstimator.addVisionMeasurement(currentPose, latency);
    }
  }
  public boolean hasTargets()
  {
    return lastResult != null && lastResult.hasTargets();
  }
  public PhotonTrackedTarget getBestTarget()
  {
    if (lastResult == null) return null;
    return lastResult.getBestTarget();
  }
  /**
   * Gets the current yaw of the target to camera.
   * REQUIRES 3D CALIBRATION.
   * @return degree value of yaw
   */
  public double getTargetYawDegrees()
  {
    assert hasTargets();
    return Math.toDegrees(getTransformToTarget().getZ());
  }
  /**
   * Gets the current pitch of the target to camera.
   * REQUIRES 3D CALIBRATION
   * @return degree value of pitch
   */
  public double getTargetPitchDegrees()
  {
    assert hasTargets();
    return Math.toDegrees(getTransformToTarget().getRotation().getY());
  }
  /**
   * Gets the current transform of the target to camera.
   * REQUIRES 3D CALIBRATION
   * @return Transform3d of robot to target
   */
  public Transform3d getTransformToTarget()
  {
    assert hasTargets();
    Transform3d transform = lastResult.getBestTarget().getBestCameraToTarget();
    return transform.plus(new Transform3d(new Translation3d(0, 0, 0),
      cameraMount.getRotation3d()));
  }
  /**
   * Gets the current distance from the target to camera.
   * REQUIRES 3D CALIBRATION
   * @return range of target in meters
   */
  public double estimateRangeMeters()
  {
    Transform3d targetTransform = getTransformToTarget();
    double x = targetTransform.getX(), y = targetTransform.getY();
    return Math.sqrt(x * x + y * y);
  }
  /**
   * Gets the odometry's current pose in meters
   * @return pose in meters
   */
  public Pose2d getCurrentPose2d()
  {
    return robotPoseEstimator.getEstimatedPosition();
  }
}
