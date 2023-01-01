// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.kauailabs.navx.frc.AHRS;

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
import frc.robot.utils.CameraMount;

import static frc.robot.RobotContainer.drivetrain;

public class PositioningSystem extends SubsystemBase {
  private final DifferentialDrivePoseEstimator robotPoseEstimator;
  private final AHRS ahrs = new AHRS();
  private final PhotonCamera camera;
  private final CameraMount mount;
  /** Creates a new PositioningSystem. */
  public PositioningSystem(PhotonCamera camera, CameraMount mount) {
    this.camera = camera;
    ahrs.reset();
    ahrs.calibrate();
    robotPoseEstimator = new DifferentialDrivePoseEstimator(ahrs.getRotation2d(), new Pose2d(),
      new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));
    this.mount = mount;
  }
  public Pose2d getRobotPose()
  {
    return robotPoseEstimator.getEstimatedPosition();
  }
  public void enableAutonomousMode()
  {
    if (camera != null) camera.setDriverMode(false);
  }
  public void enableDriverMode()
  {
    if (camera != null) camera.setDriverMode(true);
  }
  @Override
  public void periodic() {
    if (camera != null && !camera.getDriverMode())
    {
      PhotonPipelineResult result = camera.getLatestResult();
      robotPoseEstimator.update(ahrs.getRotation2d(), drivetrain.getWheelSpeeds(), drivetrain.getLeftEncoderDistance(),
        drivetrain.getRightEncoderDistance());
      if (result.hasTargets())
      {
        double imageCaptureTime = Timer.getFPGATimestamp() - result.getLatencyMillis();
        Transform3d cameraToTarget = mount == null ? result.getBestTarget().getBestCameraToTarget()
          : result.getBestTarget().getBestCameraToTarget().plus(new Transform3d(new Translation3d(), mount.getRotation3d()));
        Transform2d transform2d = new Transform2d(
          new Translation2d(cameraToTarget.getTranslation().getX(), cameraToTarget.getTranslation().getZ()),
          new Rotation2d(cameraToTarget.getRotation().getZ()));
        Pose2d targetPose = Constants.targetPoses[result.getBestTarget().getFiducialId()];
        robotPoseEstimator.addVisionMeasurement(targetPose.transformBy(transform2d.inverse()), imageCaptureTime);
      }
    }
  }
}
