package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;
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
  private final Servo rotateRatchet;
  private final Servo staticRatchet;
  private ClimbState currClimbState = ClimbState.ZEROING;
  private double set1SetPointTicks;
  private double set2SetPointTicks;
  private double shoulderSetPointTicks;
  private boolean isRotatingRatchetOn = false;
  private boolean isStaticRatchetOn = false;

  public ClimbSubsystem() {
    extendFalcon1Moveable = new TalonFX(ClimbConstants.kExtend1FalconID);
    extendFalcon2Static = new TalonFX(ClimbConstants.kExtend2FalconID);
    shoulderFalcon = new TalonSRX(ClimbConstants.kClimbShoulderId);
    rotateRatchet = new Servo(ClimbConstants.kRotateRatchetId);
    staticRatchet = new Servo(ClimbConstants.kStaticRatchetId);

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
    shoulderFalcon.configSupplyCurrentLimit(
        ClimbConstants.getShoulderCurrentLimit(), Constants.kTalonConfigTimeout);
    shoulderFalcon.enableVoltageCompensation(true);
    shoulderFalcon.setNeutralMode(NeutralMode.Brake);
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

  public void toggleStaticRatchet() {
    if (isStaticRatchetOn) staticRatchet.set(ClimbConstants.kStaticRatchetOff);
    else staticRatchet.set(ClimbConstants.kStaticRatchetOn);
    isStaticRatchetOn = !isStaticRatchetOn;
    logger.info("Setting Static Ratchet to: {}", isStaticRatchetOn);
  }

  public void toggleRotatingRatchet() {
    if (isRotatingRatchetOn) rotateRatchet.set(ClimbConstants.kRotateRatchetOff);
    else rotateRatchet.set(ClimbConstants.kRotateRatchetOn);
    isRotatingRatchetOn = !isRotatingRatchetOn;
    logger.info("Setting Rotating Ratchet to {}", isRotatingRatchetOn);
  }

  public void zeroClimb() {
    extendFalcon1Moveable.setSelectedSensorPosition(0.0);
    extendFalcon2Static.setSelectedSensorPosition(0.0);
    shoulderFalcon.setSelectedSensorPosition(0.0);
    logger.info("Zeroing all Climb axes");
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
