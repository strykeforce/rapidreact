package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.kHorizonFov;

import edu.wpi.first.math.geometry.Rotation2d;
import org.strykeforce.deadeye.TargetListTargetData;

public class HubTargetData extends TargetListTargetData {

  static int kFrameCenter = Integer.MAX_VALUE;

  /**
   * Check if this contains valid target data. Valid data is defined by the {@code valid} flag being
   * set {@code true} by the Deadeye camera and that there is one or more targets detected.
   *
   * @return true if valid
   */
  public boolean isValid() {
    return valid && !targets.isEmpty();
  }

  /**
   * Return the distance from the center of the Hub target group to the center of the camera frame.
   * You should check {@link #isValid()} before calling this method.
   *
   * @return the number of pixels, positive if ..., negative if ...
   * @throws IndexOutOfBoundsException if the list of targets is empty
   */
  public double getErrorPixels() {
    int minX = targets.get(0).topLeft.x;
    int maxX = targets.get(targets.size() - 1).bottomRight.x;
    return (maxX + minX) / 2.0 - kFrameCenter;
  }

  /**
   * Return the angle from the center of the Hub target group to the center of the camera frame. You
   * should check {@link #isValid()} before calling this method.
   *
   * @return the angle in Radians
   * @throws IndexOutOfBoundsException if the list of targets is empty
   */
  public double getErrorRadians() {
    return -kHorizonFov * getErrorPixels() / (kFrameCenter * 2);
  }

  /**
   * Return the angle from the center of the Hub target group to the center of the camera frame. You
   * should check {@link #isValid()} before calling this method.
   *
   * @return the angle as a {@code Rotation2d}
   * @throws IndexOutOfBoundsException if the list of targets is empty
   */
  public Rotation2d getErrorRotation2d() {
    return new Rotation2d(getErrorRadians());
  }
}
