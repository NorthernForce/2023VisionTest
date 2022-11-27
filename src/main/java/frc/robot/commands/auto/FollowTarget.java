// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.RobotContainer.drivetrain;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
    xController = new PIDController(0.07, 0, 0);
    SmartDashboard.putData("Rotate Controller", controller);
    SmartDashboard.putData("Drive Controller", xController);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotate = 0;
    double xSpeed = 0;
    if (trackingSystem.hasTargets())
    {
      Transform3d translation = trackingSystem.getTargetPose3d().minus(drivetrain.getPose3d());
      double range = Math.sqrt(Math.pow(translation.getX(), 2) + 
        Math.pow(translation.getY(), 2));
      rotate = controller.calculate(Math.atan(translation.getY() / translation.getX()));
      xSpeed = xController.calculate(range, distance);
      SmartDashboard.putNumber("Range", range);
    }
    drivetrain.drive(xSpeed, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    SmartDashboard.delete("Rotate - kP");
    SmartDashboard.delete("Rotate - kD");
    SmartDashboard.delete("Drive - kP");
    SmartDashboard.delete("Drive - kD");
    SmartDashboard.delete("Range");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}