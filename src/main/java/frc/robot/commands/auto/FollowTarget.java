// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.RobotContainer.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.trackingSystem;

public class FollowTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private PIDController forwardController, turnController;
  private final double distance;
  public FollowTarget(double distance) {
    addRequirements(drivetrain, trackingSystem);
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = new PIDController(0.012, 0, 0);
    forwardController = new PIDController(0.4, 0.2, 0);
    forwardController.setTolerance(0.1);
    turnController.setTolerance(0.1);
    SmartDashboard.putData("Forward Controller", forwardController);
    SmartDashboard.putData("Turn Controller", turnController);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forwardController = (PIDController)SmartDashboard.getData("Forward Controller");
    turnController = (PIDController)SmartDashboard.getData("Turn Controller");
    double rotate = 0;
    double xSpeed = 0;
    if (trackingSystem.hasTargets())
    {
      double range = trackingSystem.estimateRangeMeters();
      rotate = turnController.calculate(trackingSystem.getTargetYawDegrees(), 0);
      xSpeed = -forwardController.calculate(range, distance);
      SmartDashboard.putNumber("Forward speed", xSpeed);
      SmartDashboard.putNumber("Range", range);
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