package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Set;
import org.strykeforce.deadeye.Rect;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {
  public DeadeyeC0 shooterCamera;
  public double HORIZ_FOV = Constants.VisionConstants.HORIZ_FOV;
  // private NetworkTableInstance deadeyeNetworkTableInstance;

  public VisionSubsystem() {
    // deadeyeNetworkTableInstance = NetworkTableInstance.create();
    // deadeyeNetworkTableInstance.startClient("192.168.3.3", 1736); // TEST Deadeye
    // deadeyeNetworkTableInstance.startClient("192.168.3.3", 1735); // real Deadeye
    shooterCamera = new DeadeyeC0(); // deadeyeNetworkTableInstance);
  }

  public boolean isTargetValid() {
    return shooterCamera.getValid();
  }

  public double getHorizAngleAdjustment() {
    return Constants.VisionConstants.kHorizAngleCorrection;
  }

  public boolean isStable() {
    return true;
  }

  public ArrayList<Rect> getSortedRects() {
    ArrayList<Rect> sortedlist = new ArrayList<Rect>();
    int i;
    for (Rect Rect : shooterCamera.getTargetListData()) {
      i = 0;
      for (Rect sortedRect : sortedlist) {
        if (Rect.center().x > sortedRect.center().x) {
          i += 1;
        }
      }
      sortedlist.add(i, Rect);
    }
    return sortedlist;
  }

  public double getErrorPixels() {
    ArrayList<Rect> listData = shooterCamera.getTargetListData();
    if (listData.isEmpty()) {
      return 2767;
    }
    int minX = listData.get(0).topLeft.x;
    int maxX = listData.get(0).bottomRight.x;
    double center;
    for (int i = 0; i < listData.size(); i++) {
      if (listData.get(i).topLeft.x < minX) {
        minX = listData.get(i).topLeft.x;
      }
      if (listData.get(i).bottomRight.x > maxX) {
        maxX = listData.get(i).bottomRight.x;
      }
    }
    center = (maxX + minX) / 2;
    return center - shooterCamera.frameCenter;
  }

  public double getErrorRadians() {
    if (shooterCamera.getValid()) {
      // 57.999 * Math.max(width, height) / 1280 deg
      // 1.012 * Math.max(width, height) / 1280 rad
      return HORIZ_FOV * getErrorPixels() / (shooterCamera.frameCenter * 2) * -1;
    }
    return 2767;
  }

  public Rotation2d getErrorRotation2d() {
    return new Rotation2d(getErrorRadians());
  }

  @Override
  public Set<Measure> getMeasures() {

    return Set.of(
        new Measure("PixelOffset", () -> getErrorPixels()),
        new Measure("OffsetAngle", () -> getErrorRadians()));
  }
}
