package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import java.util.List;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class IntakeSubsystem extends MeasurableSubsystem {
  private static final Logger logger = LoggerFactory.getLogger(IntakeSubsystem.class);
  private TalonFX intakeFalcon;

  public IntakeSubsystem() {
    intakeFalcon = new TalonFX(IntakeConstants.kIntakeFalconID);
    intakeFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    intakeFalcon.configAllSettings(
        IntakeConstants.getIntakeFalconConfig(), Constants.kTalonConfigTimeout);
    intakeFalcon.enableVoltageCompensation(true);
    intakeFalcon.setNeutralMode(NeutralMode.Coast);
  }

  public void openLoopRotate(double percentOutput) {
    intakeFalcon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Intake motor turned on {}", percentOutput);
  }

  public List<BaseTalon> getTalons() {
    return List.of(intakeFalcon);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(intakeFalcon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }
}
