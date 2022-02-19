package frc.robot.subsystems;

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
    deadeye = new Deadeye<>("C0", HubTargetData.class);
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
        new Measure("Distance From Width", this::getDistance1),
        new Measure("Distance From Db", this::getDistance2),
        new Measure("Distance From Height", this::getDistance3),
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

  private double getDistance1() {
    return targetData.getDistance1();
  }

  private double getDistance2() {
    return targetData.getDistance2();
  }

  private double getDistance3() {
    return targetData.getDistance3();
  }

  private double getValid() {
    var td = targetData;
    return td.isValid() ? 1.0 : 0.0;
  }
}
