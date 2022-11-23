// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.RobotContainer.drivetrain;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.dashboard;

public class TurnToTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private PhotonCamera camera;
  private PIDController controller;
  private DoubleSupplier kP, kD;
  public TurnToTarget() {
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera = new PhotonCamera("webcam");
    camera.setPipelineIndex(1);
    controller = new PIDController(0.025, 0, 0);
    dashboard.putNumber("kP", 0.025);
    dashboard.putNumber("kD", 0);
    kP = dashboard.getSupplier("kP");
    kD = dashboard.getSupplier("kD");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.setP(kP.getAsDouble());
    controller.setD(kD.getAsDouble());
    PhotonPipelineResult result = camera.getLatestResult();
    double rotate = 0;
    if (result.hasTargets())
    {
      rotate = controller.calculate(result.getBestTarget().getYaw(), 0);
    }
    drivetrain.drive(0, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dashboard.delete("kP");
    dashboard.delete("kD");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
