// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.RobotContainer.drivetrain;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private PhotonCamera camera;
  private PIDController controller, xController;
  public TurnToTarget() {
    addRequirements(drivetrain);
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
        0.245, 0.79, 0, result.getBestTarget().getPitch());
      rotate = controller.calculate(result.getBestTarget().getYaw(), 0);
      xSpeed = xController.calculate(range, 5);
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
