package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Set;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.deadeye.Deadeye;
import org.strykeforce.deadeye.TargetDataListener;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem
    implements TargetDataListener<HubTargetData> {

  private final Deadeye<HubTargetData> deadeye;
  private volatile HubTargetData targetData = new HubTargetData();
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public VisionSubsystem() {
    NetworkTableInstance networkTableInstance = NetworkTableInstance.create();
    networkTableInstance.startClient("10.27.67.10");
    deadeye = new Deadeye<>("A0", HubTargetData.class, networkTableInstance);
    deadeye.setTargetDataListener(this);
    HubTargetData.kFrameCenter = deadeye.getCapture().width / 2;
  }

  @Override
  public void onTargetData(HubTargetData targetData) {
    this.targetData = targetData;
  }

  public HubTargetData getTargetData() {
    return targetData;
  }

  public void enable() {
    deadeye.setEnabled(true);
    logger.info("enabled vision system");
  }

  public void disable() {
    deadeye.setEnabled(false);
    logger.info("disabled vision system");
  }

  @Override
  public @NotNull Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Error Pixels", this::getErrorPixels),
        new Measure("Error Radians", this::getErrorRadians),
        new Measure("Error Degrees", this::getErrorDegrees),
        new Measure("Target Data Valid", this::getValid),
        new Measure("Test Pixel Width", this::getTargetsDistancePixel),
        new Measure("Test Ground Distance", this::getTargetsDistanceGround),
        new Measure("Target Data SN", () -> targetData.serial));
  }

  // these private getters are for the grapher and prevent a data race that can occur if targetData
  // updates after isValid() and before getErrorXXX()
  private double getErrorPixels() {
    var td = targetData;
    return td.isValid() ? td.getErrorPixels() : 2767.0;
  }

  private double getErrorRadians() {
    var td = targetData;
    return td.isValid() ? td.getErrorRadians() : 2767.0;
  }

  private double getErrorDegrees() {
    var td = targetData;
    return td.isValid() ? Math.toDegrees(td.getErrorRadians()) : 2767.0;
  }

  public double getTargetsDistancePixel() {
    var td = targetData;
    return td.isValid() ? td.testGetTargetsPixelWidth() : 2767.0;
  }

  public double getTargetsDistanceGround() {
    var td = targetData;
    return td.isValid() ? td.getGroundDistance() : 2767.0;
  }

  public double getTargetPixelWidth() {
    var td = targetData;
    return td.testGetTargetsPixelWidth();
  }

  private double getValid() {
    var td = targetData;
    return td.isValid() ? 1.0 : 0.0;
  }
}
