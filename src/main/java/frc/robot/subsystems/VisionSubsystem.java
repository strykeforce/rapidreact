package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.kHorizonFov;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import java.util.List;
import java.util.Set;
import org.strykeforce.deadeye.Rect;
import org.strykeforce.deadeye.TargetListTargetData;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {
  public DeadeyeC0 shooterCamera;
  public TargetListTargetData lastTargetData;
  // private NetworkTableInstance deadeyeNetworkTableInstance;

  public VisionSubsystem() {
    // deadeyeNetworkTableInstance = NetworkTableInstance.create();
    // deadeyeNetworkTableInstance.startClient("192.168.3.3", 1736); // TEST Deadeye
    // deadeyeNetworkTableInstance.startClient("192.168.3.3", 1735); // real Deadeye
    shooterCamera = new DeadeyeC0(); // deadeyeNetworkTableInstance);
  }

  public double getHorizAngleAdjustment() {
    return Constants.VisionConstants.kHorizAngleCorrection;
  }

  public boolean isStable() {
    return true;
  }

  public Double getErrorPixels() {
    TargetListTargetData targetData = shooterCamera.getTargetListData();
    if (!targetData.valid) return null;

    lastTargetData = targetData;
    List<Rect> listData = lastTargetData.targetsOrderedByCenterX();
    if (listData.isEmpty()) return null;

    int minX = listData.get(0).topLeft.x;
    int maxX = listData.get(listData.size() - 1).bottomRight.x;
    return Double.valueOf((maxX + minX) / 2 - shooterCamera.frameCenter);
  }

  public Double getErrorRadians() {
    // 57.999 * Math.max(width, height) / 1280 deg
    // 1.012 * Math.max(width, height) / 1280 rad
    Double pixelError = getErrorPixels();
    if (pixelError == null) return null;

    return Double.valueOf(kHorizonFov * pixelError / (shooterCamera.frameCenter * 2) * -1);
  }

  public Rotation2d getErrorRotation2d() {
    Double radianError = getErrorRadians();
    if (radianError == null) return null;

    return new Rotation2d(radianError);
  }

  @Override
  public Set<Measure> getMeasures() {

    return Set.of(
        new Measure("PixelOffset", () -> getErrorPixels()),
        new Measure("OffsetAngle", () -> getErrorRadians()));
  }
}
