package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import java.util.List;
import java.util.Set;
import org.strykeforce.deadeye.Rect;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import static frc.robot.Constants.VisionConstants.kHorizonFov;

public class VisionSubsystem extends MeasurableSubsystem {
  public DeadeyeC0 shooterCamera;
  // private NetworkTableInstance deadeyeNetworkTableInstance;

  public VisionSubsystem() {
    // deadeyeNetworkTableInstance = NetworkTableInstance.create();
    // deadeyeNetworkTableInstance.startClient("192.168.3.3", 1736); // TEST Deadeye
    // deadeyeNetworkTableInstance.startClient("192.168.3.3", 1735); // real Deadeye
    shooterCamera = new DeadeyeC0(); // deadeyeNetworkTableInstance);
  }

  public boolean isTargetValid() {
    return shooterCamera.isValid();
  }

  public double getHorizAngleAdjustment() {
    return Constants.VisionConstants.kHorizAngleCorrection;
  }

  public boolean isStable() {
    return true;
  }

  public double getErrorPixels() {
    if (shooterCamera.getTargetListData() == null) { return 2767; }
    List<Rect> listData = shooterCamera.getTargetListData().targetsOrderedByCenterX();
    if (listData.isEmpty()) { return 2767; }

    int minX = listData.get(0).topLeft.x;
    int maxX = listData.get(listData.size() - 1).bottomRight.x;
    return (maxX + minX) / 2 - shooterCamera.frameCenter;
  }

  public double getErrorRadians() {
    if (shooterCamera.isValid()) {
      // 57.999 * Math.max(width, height) / 1280 deg
      // 1.012 * Math.max(width, height) / 1280 rad
      return kHorizonFov * getErrorPixels() / (shooterCamera.frameCenter * 2) * -1;
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
