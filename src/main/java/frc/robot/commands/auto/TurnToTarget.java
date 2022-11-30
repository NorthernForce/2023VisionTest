// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.RobotContainer.drivetrain;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.trackingSystem;

public class TurnToTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private PhotonCamera camera;
  private PIDController controller;
  public TurnToTarget() {
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera = new PhotonCamera("webcam");
    camera.setPipelineIndex(1);
    SmartDashboard.putNumber("turnController - kP", 0.025);
    SmartDashboard.putNumber("turnController - kI", 0);
    SmartDashboard.putNumber("turnController - kD", 0);
    double kp = SmartDashboard.getNumber("turnController - kP", 0.025);
    double ki = SmartDashboard.getNumber("turnController - kI", 0);
    double kd = SmartDashboard.getNumber("turnController - kD", 0);
    controller = new PIDController(kp, ki, kd);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kp = SmartDashboard.getNumber("turnController - kP", 0);
    double ki = SmartDashboard.getNumber("turnController - kI", 0);
    double kd = SmartDashboard.getNumber("turnController - kD", 0);
    controller.setP(kp);
    controller.setI(ki);
    controller.setD(kd);
    PhotonPipelineResult result = camera.getLatestResult();
    double rotate = 0;
    if (result.hasTargets())
    {
      var target = trackingSystem.getTrackedTarget().getLastTarget();
      rotate = controller.calculate(target.getYaw(), 0);
    }
    drivetrain.drive(0, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
