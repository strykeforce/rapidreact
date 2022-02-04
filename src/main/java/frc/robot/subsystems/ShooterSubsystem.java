package frc.robot.subsystems;

import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ShooterSubsystem extends MeasurableSubsystem {

  private static final Logger logger = LoggerFactory.getLogger(ShooterSubsystem.class);
  // private final TalonFX shooterFalcon;
  // private final TalonFX kickerFalcon;
  // private final TalonSRX hoodTalon;

  public ShooterSubsystem() {
    // shooterFalcon = new TalonFX(ShooterConstants.kShooterFalconID);
    // shooterFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    // shooterFalcon.configAllSettings(
    //     ShooterConstants.getShooterFalconConfig(), Constants.kTalonConfigTimeout);
    // shooterFalcon.enableVoltageCompensation(true);
    // shooterFalcon.setNeutralMode(NeutralMode.Coast);

    // kickerFalcon = new TalonFX(ShooterConstants.kKickerFalconID);
    // kickerFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    // kickerFalcon.configAllSettings(
    //     ShooterConstants.getKickerFalconConfig(), Constants.kTalonConfigTimeout);
    // kickerFalcon.enableVoltageCompensation(true);
    // kickerFalcon.setNeutralMode(NeutralMode.Coast);

    // hoodTalon = new TalonSRX(ShooterConstants.kHoodTalonID);
    // hoodTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    // hoodTalon.configAllSettings(
    //     ShooterConstants.getHoodTalonConfig(), Constants.kTalonConfigTimeout);
    // hoodTalon.enableCurrentLimit(true);
    // hoodTalon.enableVoltageCompensation(true);
    // hoodTalon.setNeutralMode(NeutralMode.Coast);

    // hoodTalon.configForwardSoftLimitEnable(true);
    // hoodTalon.configForwardSoftLimitThreshold(ShooterConstants.kForwardSoftLimts);
    // hoodTalon.configReverseSoftLimitEnable(true);
    // hoodTalon.configReverseSoftLimitThreshold(ShooterConstants.kReverseSoftLimits);
  }

  public void shooterOpenLoop(double speed) {
    logger.info("Shooter on {}", speed);
    // shooterFalcon.set(ControlMode.PercentOutput, speed);
    // kickerFalcon.set(ControlMode.PercentOutput, -speed);
  }

  public void hoodOpenLoop(double speed) {
    logger.info("hood on {}", speed);
    // hoodTalon.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    // telemetryService.register(shooterFalcon);
    // telemetryService.register(kickerFalcon);
    // telemetryService.register(hoodTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }
}
