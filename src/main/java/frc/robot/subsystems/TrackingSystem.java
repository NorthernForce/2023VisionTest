// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.RobotContainer.drivetrain;

public class TrackingSystem extends SubsystemBase {
  public enum TrackingType
  {
    STATIC,
    DYNAMIC,
    NONE
  }
  public abstract class Target
  {
    public abstract void update();
    public abstract void update(PhotonTrackedTarget target);
    public abstract Pose3d getLastKnownPose();
  }
  public class StaticTarget extends Target
  {
    private Pose3d lastKnownPose;
    public StaticTarget(PhotonTrackedTarget target)
    {
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      lastKnownPose = drivetrain.getPose3d().plus(cameraToTarget);
    }
    @Override
    public Pose3d getLastKnownPose()
    {
      return lastKnownPose;
    }
    @Override
    public void update()
    {
    }
    @Override
    public void update(PhotonTrackedTarget target)
    {
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      lastKnownPose = drivetrain.getPose3d().plus(cameraToTarget);
    }
  };
  public class DynamicTarget extends Target
  {
    private Pose3d lastKnownPose;
    public DynamicTarget(PhotonTrackedTarget target)
    {
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      lastKnownPose = drivetrain.getPose3d().plus(cameraToTarget);
    }
    @Override
    public Pose3d getLastKnownPose()
    {
      return lastKnownPose;
    }
    @Override
    public void update()
    {
    }
    @Override
    public void update(PhotonTrackedTarget target)
    {
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      lastKnownPose = drivetrain.getPose3d().plus(cameraToTarget);
    }
  };
  private final PhotonCamera camera;
  private Target trackedTarget = null;
  private final TrackingType type;
  private PhotonPipelineResult lastResult;
  /** Creates a new TrackingSystem. */
  public TrackingSystem(String cameraName, TrackingType type) {
    this.type = type;
    camera = new PhotonCamera(cameraName);
  }
  @Override
  public void periodic() {
    lastResult = camera.getLatestResult();
    if (lastResult.hasTargets())
    {
      if (type != TrackingType.NONE)
      {
        if (trackedTarget == null)
        {
          if (type == TrackingType.DYNAMIC)
          {
            trackedTarget = new DynamicTarget(lastResult.getBestTarget());
          }
          else if (type == TrackingType.STATIC)
          {
            trackedTarget = new StaticTarget(lastResult.getBestTarget());
          }
        }
        else
        {
          trackedTarget.update(lastResult.getBestTarget());
        }
      }
    }
    else
    {
      if (type != TrackingType.NONE)
      {
        trackedTarget.update();
      }
    }
  }
  public boolean hasTargets()
  {
    return lastResult.hasTargets();
  }
  public PhotonTrackedTarget getBestTarget()
  {
    return lastResult.getBestTarget();
  }
  public Target getTrackedTarget()
  {
    return trackedTarget;
  }
  public Pose3d getTargetPose3d()
  {
    if (type != TrackingType.NONE)
    {
      return trackedTarget.getLastKnownPose();
    }
    if (lastResult.hasTargets())
    {
      return drivetrain.getPose3d().plus(lastResult.getBestTarget().getBestCameraToTarget());
    }
    return null;
  }
}
