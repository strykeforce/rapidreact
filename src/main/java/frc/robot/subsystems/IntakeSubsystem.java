package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MagazineConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class IntakeSubsystem extends MeasurableSubsystem {
  private static final Logger logger = LoggerFactory.getLogger(MeasurableSubsystem.class);
  // private TalonFX intakeFalcon;
  private TalonSRX intakeTalon;

  public IntakeSubsystem() {
    // intakeFalcon = new TalonFX(IntakeConstants.kIntakeFalconID);
    // intakeFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    // intakeFalcon.configAllSettings(
    //     IntakeConstants.getIntakeFalconConfig(), Constants.kTalonConfigTimeout);
    // intakeFalcon.enableVoltageCompensation(true);
    // intakeFalcon.setNeutralMode(NeutralMode.Coast);

    intakeTalon = new TalonSRX(IntakeConstants.kIntakeFalconID);
    intakeTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    intakeTalon.configAllSettings(
        MagazineConstants.getMagazineTalonConfig(), Constants.kTalonConfigTimeout);
    intakeTalon.enableCurrentLimit(true);
    intakeTalon.enableVoltageCompensation(true);
    intakeTalon.setNeutralMode(NeutralMode.Coast);
  }

  public void openLoopRotate(double percentOutput) {
    intakeTalon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Intake motor turned on {}", percentOutput);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(intakeTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }
}
