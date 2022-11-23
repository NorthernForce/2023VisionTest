// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.RobotContainer.drivetrain;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.CAMERA_PITCH_RADIANS;
import static frc.robot.Constants.CAMERA_HEIGHT_METERS;
import static frc.robot.Constants.TARGET_HEIGHT_METERS;
import static frc.robot.RobotContainer.dashboard;

public class FollowTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private PhotonCamera camera;
  private PIDController controller, xController;
  private final double distance;
  private DoubleSupplier rotateP, rotateD, driveP, driveD;
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
    dashboard.putNumber("Rotate - kP", 0.025);
    dashboard.putNumber("Rotate - kD", 0);
    dashboard.putNumber("Drive - kP", 0.07);
    dashboard.putNumber("Drive - kD", 0);
    rotateP = dashboard.getSupplier("Rotate - kP");
    rotateD = dashboard.getSupplier("Rotate - kD");
    driveP = dashboard.getSupplier("Drive - kP");
    driveD = dashboard.getSupplier("Drive - kD");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = camera.getLatestResult();
    double rotate = 0;
    double xSpeed = 0;
    controller.setP(rotateP.getAsDouble());
    xController.setP(driveP.getAsDouble());
    controller.setD(rotateD.getAsDouble());
    xController.setD(driveD.getAsDouble());
    if (result.hasTargets())
    {
      double range = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
        Units.degreesToRadians(result.getBestTarget().getPitch()));
      rotate = controller.calculate(result.getBestTarget().getYaw(), 0);
      xSpeed = xController.calculate(range, distance);
    }
    drivetrain.drive(xSpeed, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    dashboard.delete("Rotate - kP");
    dashboard.delete("Rotate - kD");
    dashboard.delete("Drive - kP");
    dashboard.delete("Drive - kD");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}