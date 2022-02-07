package frc.robot.subsystems;

import java.util.Set;
import org.jetbrains.annotations.NotNull;
import org.strykeforce.deadeye.Deadeye;
import org.strykeforce.deadeye.TargetDataListener;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem
    implements TargetDataListener<HubTargetData> {

  private final Deadeye<HubTargetData> deadeye;
  private volatile HubTargetData targetData = new HubTargetData();

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
  }

  public void disable() {
    deadeye.setEnabled(false);
  }

  @Override
  public @NotNull Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Error Pixels", () -> targetData.getErrorPixels()),
        new Measure("Error Degrees", () -> Math.toDegrees(targetData.getErrorRadians())));
  }
}
