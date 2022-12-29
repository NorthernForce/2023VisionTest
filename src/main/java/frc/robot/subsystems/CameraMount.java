// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: Calibrate/TEST
/**
 * Camera Mount Subsystem
 */
public class CameraMount extends SubsystemBase {
  /** z-axis servo */
  private final Servo zAxisRotate = new Servo(0);
  /** y-axis servo */
  private final Servo yAxisRotate = new Servo(1);
  /** Creates a camera mount */
  public CameraMount() {
  }
  /**
   * Gets the rotate value of the z-axis servo.
   * @return z-axis rotate in degrees
   */
  public double getZAxisRotateDegrees()
  {
    return zAxisRotate.getAngle();
  }
  /**
   * Gets the rotate value of the y-axis servo.
   * @return y-axis rotate in degrees
   */
  public double getYAxisRotateDegrees()
  {
    return yAxisRotate.getAngle();
  }
  /**
   * sets camera mount z-axis angle
   * @param rotate the degree value for the new angle
   */
  public void setZAxisRotateDegrees(double rotate)
  {
    zAxisRotate.setAngle(rotate);
  }
  /**
   * sets camera mount y-axis angle
   * @param rotate the degree value for the new angle
   */
  public void setYAxisRotateDegrees(double rotate)
  {
    yAxisRotate.setAngle(rotate);
  }
  /**
   * sets camera mount y-axis speed
   * @param speed the speed of the camera (between -1.0 and 1.0)
   */
  public void setYAxisSpeed(double speed)
  {
    yAxisRotate.setSpeed(speed);
  }
  /**
   * sets camera mount z-axis speed
   * @param speed the speed of the camera (between -1.0 and 1.0)
   */
  public void setZAxisSpeed(double speed)
  {
    zAxisRotate.setSpeed(speed);
  }
  /**
   * gets the camera rotation
   * @return a Rotation3d object
   */
  public Rotation3d getRotation3d()
  {
    return new Rotation3d(0, Math.toRadians(getYAxisRotateDegrees()),
      Math.toRadians(getZAxisRotateDegrees()));
  }
  @Override
  public void periodic() {
  }
}
