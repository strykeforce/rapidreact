package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ClimbSubsystem extends MeasurableSubsystem {
  private static final Logger logger = LoggerFactory.getLogger(ClimbSubsystem.class);

  private final TalonFX extendFalcon1Static;
  private final TalonFX extendFalcon2Moveable;
  private final TalonSRX shoulderFalcon;
  private ClimbState currClimbState = ClimbState.ZEROING;
  private double set1SetPointTicks;
  private double set2SetPointTicks;
  private double shoulderSetPointTicks;

  public ClimbSubsystem() {
    extendFalcon1Static = new TalonFX(Constants.ClimbConstants.kExtend1FalconID);
    extendFalcon2Moveable = new TalonFX(Constants.ClimbConstants.kExtend2FalconID);
    shoulderFalcon = new TalonSRX(Constants.ClimbConstants.kClimbShoulderId);
    configTalons();
  }

  private void configTalons() {
    extendFalcon1Static.configFactoryDefault(Constants.kTalonConfigTimeout);
    extendFalcon1Static.configAllSettings(
        ClimbConstants.getExtendFalconConfig(), Constants.kTalonConfigTimeout);
    extendFalcon1Static.enableVoltageCompensation(true);
    extendFalcon1Static.setNeutralMode(NeutralMode.Brake);

    extendFalcon2Moveable.configFactoryDefault(Constants.kTalonConfigTimeout);
    extendFalcon2Moveable.configAllSettings(
        ClimbConstants.getExtendFalconConfig(), Constants.kTalonConfigTimeout);
    extendFalcon2Moveable.enableVoltageCompensation(true);
    extendFalcon2Moveable.setNeutralMode(NeutralMode.Brake);

    shoulderFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    shoulderFalcon.configAllSettings(
        ClimbConstants.getShoulderTalonConfig(), Constants.kTalonConfigTimeout);
    shoulderFalcon.enableVoltageCompensation(true);
    shoulderFalcon.setNeutralMode(NeutralMode.Coast);
  }

  public void openLoopShoulder(double speed) {
    logger.info("Shoulder is rotating: {} ticks per 100 ms", speed);
    shoulderFalcon.set(ControlMode.PercentOutput, speed);
  }

  public void openLoopSet1Static(double speed) {
    logger.info("Set 1 Static arms: {} ticks per 100 ms", speed);
    extendFalcon1Static.set(ControlMode.PercentOutput, speed);
  }

  public void openLoopSet2Moveable(double speed) {
    logger.info("Set 2 Moveable arms: {} ticks per 100 ms", speed);
    extendFalcon2Moveable.set(ControlMode.PercentOutput, speed);
  }

  public void offsetShoulder(double offset) {
    rotateShoulder(shoulderSetPointTicks + offset);
  }

  public void rotateShoulder(double setPointsTicks) {
    logger.info("Shoulder is rotating to {} ticks", setPointsTicks);
    shoulderSetPointTicks = setPointsTicks;
    shoulderFalcon.set(ControlMode.MotionMagic, setPointsTicks);
  }

  public void acuateLoopSet1(double setPointTicks) {
    logger.info("Static arm set 1 is going to {} ticks", setPointTicks);
    set1SetPointTicks = setPointTicks;
    extendFalcon1Static.set(ControlMode.MotionMagic, setPointTicks);
  }

  public void acuateLoopSet2(double setPointTicks) {
    logger.info("Moveable arm Set 2 is going to {} ticks", setPointTicks);
    set2SetPointTicks = setPointTicks;
    extendFalcon2Moveable.set(ControlMode.MotionMagic, setPointTicks);
  }

  public void zeroShoulder() {
    switch (currClimbState) {
      case ZEROING:
        shoulderFalcon.configContinuousCurrentLimit(1, 3);
        shoulderFalcon.set(ControlMode.PercentOutput, -0.2);
        double velocity = shoulderFalcon.getSelectedSensorVelocity();
        if (velocity <= 10) {
          currClimbState = ClimbState.ZEROED;
        }
      case ZEROED:
        {
          shoulderFalcon.configContinuousCurrentLimit(
              Constants.ClimbConstants.getShoulderTalonConfig().continuousCurrentLimit,
              Constants.ClimbConstants.getShoulderTalonConfig().peakCurrentDuration);
          shoulderFalcon.set(ControlMode.PercentOutput, 0);
        }
    }
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(extendFalcon1Static);
    telemetryService.register(extendFalcon2Moveable);
    telemetryService.register(shoulderFalcon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public enum ClimbState {
    ZEROING,
    ZEROED
  }
}
