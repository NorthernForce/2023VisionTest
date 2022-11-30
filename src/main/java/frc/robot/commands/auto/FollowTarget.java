// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.RobotContainer.drivetrain;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.trackingSystem;

public class FollowTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private PhotonCamera camera;
  private PIDController controller, xController;
  private final double distance;
  public FollowTarget(double distance) {
    addRequirements(drivetrain, trackingSystem);
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera = new PhotonCamera("webcam");
    camera.setPipelineIndex(1);
    controller = new PIDController(0.025, 0, 0);
    xController = new PIDController(0.1, 0.01, 0);
    xController.setTolerance(0.1);
    SmartDashboard.putData("Rotate Controller", controller);
    SmartDashboard.putData("Drive Controller", xController);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotate = 0;
    double xSpeed = 0;
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets())
    {
      PhotonTrackedTarget target = result.getBestTarget();
      double range = PhotonUtils.calculateDistanceToTargetMeters(
        Constants.CAMERA_HEIGHT_METERS,
        Constants.TARGET_HEIGHT_METERS,
        Constants.CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(target.getPitch())
      );
      rotate = controller.calculate(target.getYaw(), 0);
      xSpeed = -xController.calculate(range, distance);
      SmartDashboard.putNumber("speed", xSpeed);
      SmartDashboard.putNumber("Range", range);
      SmartDashboard.putNumber("distane", distance);
    }
    drivetrain.drive(xSpeed, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}