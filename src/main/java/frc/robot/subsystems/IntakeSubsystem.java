package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
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
  private IntakeState currIntakeState = IntakeState.STOP;
  private MagazineSubsystem magazineSubsystem;

  public IntakeSubsystem() {
    intakeFalcon = new TalonFX(IntakeConstants.kIntakeFalconID);
    intakeFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    intakeFalcon.configAllSettings(
        IntakeConstants.getIntakeFalconConfig(), Constants.kTalonConfigTimeout);
    intakeFalcon.enableVoltageCompensation(true);
    intakeFalcon.setNeutralMode(NeutralMode.Coast);
  }

  public void setMagazineSubsystem(MagazineSubsystem magazineSubsystem) {
    this.magazineSubsystem = magazineSubsystem;
  }

  public void openLoopRotate(double percentOutput) {
    if (percentOutput > 0) {
      logger.info("Intake: {} -> INTAKING", currIntakeState);
      currIntakeState = IntakeState.INTAKING;
    } else if ((percentOutput < 0) && (percentOutput != IntakeConstants.kIntakeEjectSpeed)) {
      logger.info("Intake: {} -> REVERSE", currIntakeState);
      currIntakeState = IntakeState.REVERSE;
    } else if ((percentOutput == IntakeConstants.kIntakeEjectSpeed)) {
      logger.info("Intake: {} -> EJECT", currIntakeState);
      currIntakeState = IntakeState.EJECT;
    } else {
      logger.info("Intake: {} -> IDLE", currIntakeState);
      currIntakeState = IntakeState.IDLE;
    }
    intakeFalcon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Intake falcon turned on {}", percentOutput);
  }

  public List<BaseTalon> getTalons() {
    return List.of(intakeFalcon);
  }

  @Override
  public void periodic() {
    switch (currIntakeState) {
      case INTAKING:
        break;
      case EJECT:
        break;
      case REVERSE:
        if (!magazineSubsystem.isMagazineFull()) {
          logger.info("Intake: Magazine Not Full, Re-Enabling Intake.");
          openLoopRotate(
              DriverStation.isAutonomous()
                  ? IntakeConstants.kIntakeSpeedAuto
                  : IntakeConstants.kIntakeSpeed);
        }
        break;
      case IDLE:
        break;
      case STOP:
        break;
    }
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

  public enum IntakeState {
    INTAKING,
    EJECT,
    REVERSE,
    IDLE,
    STOP;
  }
}
