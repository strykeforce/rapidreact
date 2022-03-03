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

  private final TalonFX extendFalcon1Moveable;
  private final TalonFX extendFalcon2Static;
  private final TalonSRX shoulderFalcon;
  private ClimbState currClimbState = ClimbState.ZEROING;
  private double set1SetPointTicks;
  private double set2SetPointTicks;
  private double shoulderSetPointTicks;

  public ClimbSubsystem() {
    extendFalcon1Moveable = new TalonFX(Constants.ClimbConstants.kExtend1FalconID);
    extendFalcon2Static = new TalonFX(Constants.ClimbConstants.kExtend2FalconID);
    shoulderFalcon = new TalonSRX(Constants.ClimbConstants.kClimbShoulderId);
    configTalons();
  }

  private void configTalons() {
    extendFalcon1Moveable.configFactoryDefault(Constants.kTalonConfigTimeout);
    extendFalcon1Moveable.configAllSettings(
        ClimbConstants.getExtendFalconConfig(), Constants.kTalonConfigTimeout);
    extendFalcon1Moveable.enableVoltageCompensation(true);
    extendFalcon1Moveable.setNeutralMode(NeutralMode.Brake);

    extendFalcon2Static.configFactoryDefault(Constants.kTalonConfigTimeout);
    extendFalcon2Static.configAllSettings(
        ClimbConstants.getExtendFalconConfig(), Constants.kTalonConfigTimeout);
    extendFalcon2Static.enableVoltageCompensation(true);
    extendFalcon2Static.setNeutralMode(NeutralMode.Brake);

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

  public void openLoopSet1Moveable(double speed) {
    logger.info("Set 1 Static arms: {} ticks per 100 ms", speed);
    extendFalcon1Moveable.set(ControlMode.PercentOutput, speed);
  }

  public void openLoopSet2Static(double speed) {
    logger.info("Set 2 Moveable arms: {} ticks per 100 ms", speed);
    extendFalcon2Static.set(ControlMode.PercentOutput, speed);
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
    logger.info("Moveable arm set 1 is going to {} ticks", setPointTicks);
    set1SetPointTicks = setPointTicks;
    extendFalcon1Moveable.set(ControlMode.MotionMagic, setPointTicks);
  }

  public void acuateLoopSet2(double setPointTicks) {
    logger.info("Static arm Set 2 is going to {} ticks", setPointTicks);
    set2SetPointTicks = setPointTicks;
    extendFalcon2Static.set(ControlMode.MotionMagic, setPointTicks);
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
    telemetryService.register(extendFalcon1Moveable);
    telemetryService.register(extendFalcon2Static);
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
