package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.Set;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class PowerDistHub extends MeasurableSubsystem {
  private final PowerDistribution powerDistribution;
  private final Timer timer;
  private boolean isRebooting = false;

  public PowerDistHub() {
    powerDistribution = new PowerDistribution(1, ModuleType.kRev);
    timer = new Timer();
    powerDistribution.setSwitchableChannel(true);
  }

  public void powerCycleDeadeye() {
    isRebooting = true;
    timer.reset();
    timer.start();
    powerDistribution.setSwitchableChannel(false);
  }

  @Override
  public void periodic() {
    if (isRebooting && timer.hasElapsed(Constants.kDeadeyePowerCycleTimeout)) {
      powerDistribution.setSwitchableChannel(true);
      isRebooting = false;
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }
}
