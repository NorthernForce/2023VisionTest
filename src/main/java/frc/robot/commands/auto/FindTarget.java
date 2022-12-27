// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.cameraMount;
import static frc.robot.RobotContainer.trackingSystem;

public class FindTarget extends CommandBase {
  private double speed;
  /** Creates a new TurnCamera. */
  public FindTarget(double speed) {
    this.speed = speed;
    addRequirements(cameraMount, trackingSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cameraMount.setYAxisRotate(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (cameraMount.getZAxisRotate() > 250 || cameraMount.getZAxisRotate() < 20)
    {
      speed = -speed;
    }
    cameraMount.setZAxisSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cameraMount.setZAxisSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trackingSystem.hasTargets();
  }
}
