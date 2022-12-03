// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.RobotContainer.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.trackingSystem;

public class TurnToTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private PIDController controller;
  public TurnToTarget() {
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = new PIDController(0.025, 0, 0);
    SmartDashboard.putData("Turn Controller", controller);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller = (PIDController)SmartDashboard.getData("Turn Controller");
    double rotate = 0;
    if (trackingSystem.hasTargets())
    {
      double rotation = trackingSystem.getTargetYawDegrees();
      rotate = controller.calculate(rotation, 0);
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
