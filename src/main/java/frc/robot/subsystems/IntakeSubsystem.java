package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
  private TalonSRX intakeExtendTalon;
  private double intakeSetPointTicks;
  private int lastAbsPos = Integer.MAX_VALUE;
  private int zeroStableCounts = 0;
  private boolean didZero = false;
  private boolean isIntakeExtended = false;

  public IntakeSubsystem() {
    intakeFalcon = new TalonFX(IntakeConstants.kIntakeFalconID);
    intakeFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    intakeFalcon.configAllSettings(
        IntakeConstants.getIntakeFalconConfig(), Constants.kTalonConfigTimeout);
    intakeFalcon.enableVoltageCompensation(true);
    intakeFalcon.setNeutralMode(NeutralMode.Coast);

    intakeExtendTalon = new TalonSRX(IntakeConstants.kIntakeExtendTalonID);
    intakeExtendTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    intakeExtendTalon.configAllSettings(
        IntakeConstants.getIntakeExtendTalonConfig(), Constants.kTalonConfigTimeout);
    intakeExtendTalon.configSupplyCurrentLimit(
        IntakeConstants.getIntakeExtendCurrentLimit(), Constants.kTalonConfigTimeout);
    intakeExtendTalon.enableVoltageCompensation(true);
    intakeExtendTalon.setNeutralMode(NeutralMode.Brake);

    zeroIntake();
  }

  public void zeroIntake() {
    int absPos = intakeExtendTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    if (Math.abs(absPos - lastAbsPos) <= IntakeConstants.kZeroStableBand) zeroStableCounts++;
    else zeroStableCounts = 0;
    lastAbsPos = absPos;

    if (zeroStableCounts > IntakeConstants.kZeroStableCounts) {
      int offset = absPos - IntakeConstants.kIntakeZeroTicks;
      intakeExtendTalon.setSelectedSensorPosition(offset);
      logger.info(
          "Intake zeroed; offset: {} zeroTicks: {} absPosition: {}",
          offset,
          IntakeConstants.kIntakeZeroTicks,
          absPos);
      didZero = true;
      retractClosedLoop();
    }
  }

  public void openLoopRotate(double percentOutput) {
    intakeFalcon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Intake falcon turned on {}", percentOutput);
  }

  public void extendClosedLoop() {
    intakeExtendTalon.set(ControlMode.MotionMagic, IntakeConstants.kIntakeExtendPos);
    intakeSetPointTicks = IntakeConstants.kIntakeExtendPos;
    isIntakeExtended = true;
    logger.info("Intake is extending to {}", IntakeConstants.kIntakeExtendPos);
  }

  public void retractClosedLoop() {
    intakeExtendTalon.set(ControlMode.MotionMagic, IntakeConstants.kIntakeRetractPos);
    intakeSetPointTicks = IntakeConstants.kIntakeRetractPos;
    isIntakeExtended = false;
    logger.info("Intake is retracting to {}", IntakeConstants.kIntakeRetractPos);
  }

  public boolean getIsIntakeExtended() {
    return isIntakeExtended;
  }

  public boolean isIntakeAtPos() {
    return (Math.abs(intakeSetPointTicks - intakeExtendTalon.getSelectedSensorPosition())
        < IntakeConstants.kCloseEnoughTicks);
  }

  @Override
  public void periodic() {
    if (!didZero) zeroIntake();
  }

  public List<BaseTalon> getTalons() {
    return List.of(intakeFalcon);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(intakeFalcon);
    telemetryService.register(intakeExtendTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }
}
