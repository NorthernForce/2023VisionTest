// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.auto.FollowTarget;
import frc.robot.commands.auto.TurnToTarget;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TrackingSystem;
import frc.robot.subsystems.TrackingSystem.TrackingType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = new Drivetrain();

  private final OI oi = new OI();
  public static final Dashboard dashboard = new Dashboard();
  public static final TrackingSystem trackingSystem = new TrackingSystem("webcam",
    TrackingType.STATIC);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initDefaultCommands();
    oi.bindButtons();
    dashboard.clearDashboard();
    dashboard.displayCommands(new TurnToTarget(), new FollowTarget(4), new InstantCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return dashboard.getSelection();
  }

  private void initDefaultCommands() {
    drivetrain.setDefaultCommand(new DriveWithJoystick());
  }
}
