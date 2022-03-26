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
    intakeExtendTalon.enableVoltageCompensation(true);
    intakeExtendTalon.setNeutralMode(NeutralMode.Brake);

    zeroIntake();
    intakeRetractClosedLoop();
  }

  public void zeroIntake() {
    int absPos = intakeExtendTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    // inverted because absolute and relative encoders are out of phase
    int offset = absPos - IntakeConstants.kIntakeZeroTicks;
    intakeExtendTalon.setSelectedSensorPosition(offset);
    logger.info(
        "Intake zeroed; offset: {} zeroTicks: {} absPosition: {}",
        offset,
        IntakeConstants.kIntakeZeroTicks,
        absPos);
  }

  public void openLoopRotate(double percentOutput) {
    intakeFalcon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Intake falcon turned on {}", percentOutput);
  }

  public void intakeExtendClosedLoop() {
    intakeExtendTalon.set(ControlMode.MotionMagic, IntakeConstants.kIntakeExtendPos);
    intakeSetPointTicks = IntakeConstants.kIntakeExtendPos;
    logger.info("Intake is extending to {}", IntakeConstants.kIntakeExtendPos);
  }

  public void intakeRetractClosedLoop() {
    intakeExtendTalon.set(ControlMode.MotionMagic, IntakeConstants.kIntakeRetractPos);
    intakeSetPointTicks = IntakeConstants.kIntakeRetractPos;
    logger.info("Intake is retracting to {}", IntakeConstants.kIntakeRetractPos);
  }

  public boolean isIntakeAtPos() {
    return (Math.abs(intakeSetPointTicks - intakeExtendTalon.getSelectedSensorPosition())
        < IntakeConstants.kCloseEnoughTicks);
  }

  public List<BaseTalon> getTalons() {
    return List.of(intakeFalcon, intakeExtendTalon);
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
