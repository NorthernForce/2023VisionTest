// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.RobotContainer.drivetrain;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.CAMERA_PITCH_RADIANS;
import static frc.robot.Constants.CAMERA_HEIGHT_METERS;
import static frc.robot.Constants.TARGET_HEIGHT_METERS;

public class FollowTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private PhotonCamera camera;
  private PIDController controller, xController;
  private final double distance;
  public FollowTarget(double distance) {
    addRequirements(drivetrain);
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera = new PhotonCamera("webcam");
    controller = new PIDController(0.025, 0, 0);
    xController = new PIDController(0.07, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = camera.getLatestResult();
    double rotate = 0;
    double xSpeed = 0;
    if (result.hasTargets())
    {
      double range = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
        result.getBestTarget().getPitch());
      rotate = controller.calculate(result.getBestTarget().getYaw(), 0);
      xSpeed = xController.calculate(range, distance);
    }
    drivetrain.drive(xSpeed, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}