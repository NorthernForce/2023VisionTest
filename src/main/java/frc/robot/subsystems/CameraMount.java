package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Servo;

// TODO: Calibrate/TEST
/**
 * Camera Mount
 */
public class CameraMount {
  /** z-axis servo */
  private final Servo zAxisRotate;
  /** y-axis servo */
  private final Servo yAxisRotate;
  /** Creates a camera mount */
  public CameraMount(int yAxis, int zAxis) {
    yAxisRotate = new Servo(yAxis);
    zAxisRotate = new Servo(zAxis);
  }
  /**
   * Gets the rotate value of the z-axis servo.
   * @return z-axis rotate in degrees
   */
  public double getZAxisRotateDegrees()
  {
    return zAxisRotate.getAngle() - 90;
  }
  /**
   * Gets the rotate value of the y-axis servo.
   * @return y-axis rotate in degrees
   */
  public double getYAxisRotateDegrees()
  {
    return yAxisRotate.getAngle() - 90;
  }
  /**
   * sets camera mount z-axis angle
   * @param rotate the degree value for the new angle
   */
  public void setZAxisRotateDegrees(double rotate)
  {
    zAxisRotate.setAngle(rotate + 90);
  }
  /**
   * sets camera mount y-axis angle
   * @param rotate the degree value for the new angle
   */
  public void setYAxisRotateDegrees(double rotate)
  {
    yAxisRotate.setAngle(rotate + 90);
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
}
