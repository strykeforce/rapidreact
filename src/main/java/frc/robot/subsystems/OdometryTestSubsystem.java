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

  // Position Coordinates of odometry tape sections
  // Pos 1: (0.46, 4.11)
  // Pos 2: (7.0, 0.46)
  // Pos 3: (5.0, 7.77)
  // Pos 4: (12.2, 6.83)

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("Odometry Test Location", () -> odometryPos));
  }
}
