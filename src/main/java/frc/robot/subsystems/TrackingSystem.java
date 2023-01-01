// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CameraMount;

public class TrackingSystem extends SubsystemBase {
  private final PhotonCamera camera;
  private PhotonPipelineResult lastResult;
  private final CameraMount cameraMount;
  /** Creates a new TrackingSystem. */
  public TrackingSystem(PhotonCamera camera, CameraMount mount) {
    this.camera = camera;
    cameraMount = mount;
  }
  @Override
  public void periodic() {
    if (camera != null) lastResult = camera.getLatestResult();
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
  public void enableAutonomousMode()
  {
    if (camera != null) camera.setDriverMode(false);
  }
  public void enableDriverMode()
  {
    if (camera != null) camera.setDriverMode(true);
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
    if (!hasTargets()) return null;
    System.out.println(lastResult.getBestTarget());
    Transform3d transform = lastResult.getBestTarget().getBestCameraToTarget();
    return cameraMount == null ? transform : transform.plus(new Transform3d(new Translation3d(0, 0, 0),
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
}
