package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class Conversions {

  /**
   * @param rotations    CANCoder Counts
   * @return Degrees of Rotation of Mechanism
   */
  public static double rotationsToDegrees(double rotations) {
    return rotations * (360.0);
  }

  /**
   * @param rotations    CANCoder Counts
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double rotationsToDegrees(double rotations, double gearRatio) {
    return rotations * (360.0 / (gearRatio));
  }

  /**
   * @param degrees   Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return CANCoder Counts
   */
  public static double degreesToRotations(double degrees, double gearRatio) {
    double ticks = degrees / (360.0 / (gearRatio));
    return ticks;
  }

    /**
   * @param degrees   Degrees of rotation of Mechanism
   * @return Rotation Counts
   */
  public static double degreesToRotations(double degrees) {
    double ticks = degrees / 360.0;
    return ticks;
  }


  /**
   * @param velocitycounts Falcon Velocity Counts
   * @param circumference  Circumference of Wheel
   * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
   *                       Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double RPSToMPS(double velocitycounts, double circumference, double gearRatio) {
    double wheelRPS = velocitycounts/ gearRatio;
    double wheelMPS = (wheelRPS * circumference) / 60;
    return wheelMPS;
  }

  /**
   * @param rotationCounts Falcon position Counts
   * @param circumference  Circumference of Wheel
   * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
   *                       Falcon RPM)
   * @return Meters
   */
  public static double rotationsToMeters(double rotationCounts, double circumference, double gearRatio) {
    double wheelR = rotationCounts/ gearRatio;
    double wheelM = (wheelR * circumference);
    return wheelM;
  }

  /**
   * @param velocity      Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio     Gear Ratio between Falcon and Mechanism (set to 1 for
   *                      Falcon RPM)
   * @return Rotations per second
   */
  public static double MPSToRPS(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = wheelRPM/ gearRatio;
    return wheelVelocity;
  }

  /**
   * 
   * @param value
   * @param gearRatio 
   * @return
   */
  public static double gearRatioConvert(double value, double gearRatio) {
    return value / gearRatio;
  }

  /**
   * @param x position from joystick
   * @param y position from joystick
   * @return Rotation2d
   * 
   *         <pre>
      Joystick input:
               (y:-)
                 |
                 |
       (x:-)-----|------(x:+)
                 |
                 |
               (y:+)
  
   intial angle (-180 -> 180):
             -180/180°
               (x:-)
                 |
                 |
   -90°(y:-)-----|------(y:+) 90°
                 | ノ
                 |   θ
               (x:+)
                0°
  
  transformed(+180) desired angle (0 -> 360): same angle as gyro
                 0°
               (x:+)
             θ   |
               / |
    90°(y:+)-----|------(y:-)270°
                 | 
                 |  
               (x:-)
                180°
   *         </pre>
   */
  public static Rotation2d ConvertJoystickToAngle(double x, double y) {
    return new Rotation2d(Math.atan2(x, y) + Math.PI);
  }

  /**
   * 
   * Maps the joystick values from a circle to a square.
   * Otherwise the joystick hardware reads values in a square from (-1, -1) to (1,
   * 1), but because of the circular joystick shape you can never reach the
   * corners of this square.
   * 
   * @param returnAxis The axis desired to be converted
   * @param otherAxis  the other axsis (need for computation)
   * @return
   */
  public static Double mapJoystick(Double returnAxis, double otherAxis) {
    return returnAxis * Math.sqrt(1 - ((otherAxis * otherAxis) / 2));
  }
}