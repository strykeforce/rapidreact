package frc.robot.subsystems;

import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class OdometryTestSubsystem extends MeasurableSubsystem {
  private double odometryPos;

  public OdometryTestSubsystem() {}

  public void setPos(double pos) {
    odometryPos = pos;
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("My Measure", () -> odometryPos));
  }
}
