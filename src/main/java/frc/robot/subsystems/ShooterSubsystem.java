package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ShooterSubsystem extends MeasurableSubsystem {

  private static final Logger logger = LoggerFactory.getLogger(ShooterSubsystem.class);
  private final TalonFX shooterFalcon;
  private final TalonFX kickerFalcon;
  private final TalonSRX hoodTalon;
  private final MagazineSubsystem magazineSubsystem;
  private ShooterState currentState = ShooterState.STOP;
  private double shooterSetPointTicks, kickerSetpointTicks, hoodSetPointTicks;

  public ShooterSubsystem(MagazineSubsystem magazineSubsystem) {
    this.magazineSubsystem = magazineSubsystem;
    shooterFalcon = new TalonFX(ShooterConstants.kShooterFalconID);
    shooterFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    shooterFalcon.configAllSettings(
        ShooterConstants.getShooterFalconConfig(), Constants.kTalonConfigTimeout);
    shooterFalcon.enableVoltageCompensation(true);
    shooterFalcon.setNeutralMode(NeutralMode.Coast);

    kickerFalcon = new TalonFX(ShooterConstants.kKickerFalconID);
    kickerFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    kickerFalcon.configAllSettings(
        ShooterConstants.getKickerFalconConfig(), Constants.kTalonConfigTimeout);
    kickerFalcon.enableVoltageCompensation(true);
    kickerFalcon.setNeutralMode(NeutralMode.Coast);

    hoodTalon = new TalonSRX(ShooterConstants.kHoodTalonID);
    hoodTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    hoodTalon.configAllSettings(
        ShooterConstants.getHoodTalonConfig(), Constants.kTalonConfigTimeout);
    hoodTalon.enableCurrentLimit(true);
    hoodTalon.enableVoltageCompensation(true);
    hoodTalon.setNeutralMode(NeutralMode.Coast);

    hoodTalon.configForwardSoftLimitEnable(true);
    hoodTalon.configForwardSoftLimitThreshold(ShooterConstants.kForwardSoftLimts);
    hoodTalon.configReverseSoftLimitEnable(true);
    hoodTalon.configReverseSoftLimitThreshold(ShooterConstants.kReverseSoftLimits);
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

  public void shooterClosedLoop(double kickerSpeed, double shooterSpeed) {
    shooterFalcon.set(ControlMode.Velocity, shooterSpeed);
    kickerFalcon.set(ControlMode.Velocity, kickerSpeed);
    shooterSetPointTicks = shooterSpeed;
    kickerSetpointTicks = kickerSpeed;
    logger.info("Kicker at {} speed, Shooter at {} speed", kickerSpeed, shooterSpeed);
  }

  public void hoodClosedLoop(double hoodPos) {
    hoodTalon.set(ControlMode.MotionMagic, hoodPos);
    hoodSetPointTicks = hoodPos;
    logger.info("Hood is moving to {}", hoodPos);
  }

  public boolean isShooterAtSpeed() {
    return (Math.abs(shooterSetPointTicks - shooterFalcon.getSelectedSensorVelocity())
            < ShooterConstants.kCloseEnoughTicksP100ms
        && Math.abs(kickerSetpointTicks - kickerFalcon.getSelectedSensorVelocity())
            < ShooterConstants.kCloseEnoughTicksP100ms);
  }

  public boolean isHoodAtPos() {
    return (Math.abs(hoodSetPointTicks - hoodTalon.getSelectedSensorPosition())
        < ShooterConstants.kCloseEnoughTicks);
  }

  public ShooterState getCurrentState() {
    return currentState;
  }

  public void arm() {
    currentState = ShooterState.ARMING;
    shooterClosedLoop(ShooterConstants.kKickerArmSpeed, ShooterConstants.kShooterArmSpeed);
    logger.info("Arming starting");
  }

  public void shoot() {
    currentState = ShooterState.ADJUSTING;
    if (magazineSubsystem.isNextCargoAlliance()) {
      //Create look up table
    } else {
      shooterClosedLoop(ShooterConstants.kKickerOpSpeed , ShooterConstants.kShooterOpSpeed);
      hoodClosedLoop(ShooterConstants.kHoodOpPos);
    }
  }

  public void stop() {
    logger.info("{} -> STOP", currentState);
    currentState = ShooterState.STOP;
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case STOP:
        if (kickerFalcon.getMotorOutputPercent() != 0.0
            || shooterFalcon.getMotorOutputPercent() != 0.0) {
          shooterOpenLoop(0.0);
        }
        break;
      case ARMING:
        if (isShooterAtSpeed()) {
          currentState = ShooterState.ARMED;
          logger.info("ARMING -> ARMED");
        }
        break;
      case ARMED:
        // Just indicates that it is ready to shoot for other classes       
        break;
      case ADJUSTING:
      if (isHoodAtPos() && isShooterAtSpeed()) {
        currentState = ShooterState.SHOOT;
        logger.info("ADJUSTING -> SHOOT");
      }
        break;
      case SHOOT:
        // Just a state that lets other classes it needs to shoot
        break;
    }
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

  enum ShooterState {
    STOP,
    ARMING,
    ARMED,
    ADJUSTING,
    SHOOT;
  }
}
