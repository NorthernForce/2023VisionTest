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
  public abstract class Target
  {
    public abstract void update();
    public abstract void update(PhotonTrackedTarget target);
    public abstract Pose3d getLastKnownPose();
    public abstract PhotonTrackedTarget getLastTarget();
  }
  public class StaticTarget extends Target
  {
    private Pose3d lastKnownPose;
    private PhotonTrackedTarget lastTarget;
    public StaticTarget(PhotonTrackedTarget target)
    {
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      lastKnownPose = drivetrain.getPose3d().plus(cameraToTarget);
      lastTarget = target;
    }
    @Override
    public Pose3d getLastKnownPose()
    {
      return lastKnownPose;
    }
    @Override
    public PhotonTrackedTarget getLastTarget()
    {
      return lastTarget;
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
      lastTarget = target;
    }
  };
  public class DynamicTarget extends Target
  {
    private Pose3d lastKnownPose;
    private PhotonTrackedTarget lastTarget;
    public DynamicTarget(PhotonTrackedTarget target)
    {
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      lastKnownPose = drivetrain.getPose3d().plus(cameraToTarget);
      lastTarget = target;
    }
    @Override
    public Pose3d getLastKnownPose()
    {
      return lastKnownPose;
    }
    @Override
    public PhotonTrackedTarget getLastTarget()
    {
      return lastTarget;
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
      lastTarget = target;
    }
  };
  private final PhotonCamera camera;
  private Target trackedTarget = null;
  private final TrackingType type;
  private PhotonPipelineResult lastResult;
  /** Creates a new TrackingSystem. */
  public TrackingSystem(String cameraName, TrackingType type, CameraFilter filter) {
    this.type = type;
    this.camera = new PhotonCamera(cameraName);
    camera.setPipelineIndex(filter.getPipelineIndex());
  }
  void setFilter(CameraFilter filter)
  {
    camera.setPipelineIndex(filter.getPipelineIndex());
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
    return trackedTarget != null;
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
