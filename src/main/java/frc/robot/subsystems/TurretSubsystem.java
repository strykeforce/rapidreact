package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import frc.robot.Constants;
import java.util.List;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class TurretSubsystem extends MeasurableSubsystem {
  private static final int TURRET_ID = 42; // move to constants file
  private static final double TURRET_TICKS_PER_DEGREE =
      Constants.TurretConstants.TURRET_TICKS_PER_DEGREE;
  private static final double kWrapRange = Constants.TurretConstants.kWrapRange;
  private static final double kTurretMidpoint = Constants.TurretConstants.kTurretMidpoint;
  public static boolean talonReset;
  private static TalonSRX turret;
  private static double targetTurretPosition = 0;
  private static int turretStableCounts = 0;
  private static int kTurretZeroTicks;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public TurretSubsystem() {
    kTurretZeroTicks = Constants.TurretConstants.kTurretZeroTicks;
    configTalons();
  }

  private void configTalons() {
    TalonSRXConfiguration turretConfig = new TalonSRXConfiguration();

    // turret setup
    turret = new TalonSRX(TURRET_ID);
    turretConfig.forwardSoftLimitThreshold = Constants.TurretConstants.kForwardLimit;
    turretConfig.reverseSoftLimitThreshold = Constants.TurretConstants.kReverseLimit;
    turretConfig.forwardSoftLimitEnable = true;
    turretConfig.reverseSoftLimitEnable = true;
    turretConfig.slot0.kP = 2;
    turretConfig.slot0.kI = 0.01;
    turretConfig.slot0.kD = 80;
    turretConfig.slot0.kF = 0.21;
    turretConfig.slot0.integralZone = 40;
    turretConfig.slot0.maxIntegralAccumulator = 4500;
    turretConfig.voltageMeasurementFilter = 32;
    turretConfig.voltageCompSaturation = 12;
    turretConfig.motionCruiseVelocity = 4_000;
    turretConfig.motionAcceleration = 30_000;
    turretConfig.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled;
    turretConfig.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;
    turret.configAllSettings(turretConfig);
    turret.setNeutralMode(NeutralMode.Brake);
    turret.enableCurrentLimit(false);
    turret.enableVoltageCompensation(true);
    turret.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5, 30, 500));
  }

  public List<BaseTalon> getTalons() {
    return List.of(turret);
  }

  public void rotateTurret(double offsetDegrees) {
    double currentAngle = turret.getSelectedSensorPosition() / TURRET_TICKS_PER_DEGREE;
    double targetAngle =
        currentAngle + offsetDegrees + Constants.VisionConstants.kHorizAngleCorrection;
    if (targetAngle <= kWrapRange && turret.getSelectedSensorPosition() > kTurretMidpoint
        || targetAngle < 0) {
      targetAngle += 360;
    }
    double setPoint = targetAngle * TURRET_TICKS_PER_DEGREE;
    setTurret(setPoint);
  }

  public void openLoopRotate(double percentOutput) {
    turret.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setTurret(double setPointTicks) {
    if (turret.hasResetOccurred()) {
      talonReset = true;
    } else {
      targetTurretPosition = setPointTicks;
      turret.set(ControlMode.MotionMagic, setPointTicks);
      logger.info("Rotating Turret to {} ticks", setPointTicks);
    }
  }

  public void findTarget(double error) {}

  public boolean turretAtTarget() {
    double currentTurretPosition = turret.getSelectedSensorPosition();
    if (!Constants.isCompBot
        || Math.abs(targetTurretPosition - currentTurretPosition)
            > Constants.TurretConstants.kCloseEnoughTurret) {
      turretStableCounts = 0;
    } else {
      turretStableCounts++;
    }
    if (turretStableCounts >= Constants.ShooterConstants.kStableCounts) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public Set<Measure> getMeasures() {

    return Set.of(new Measure("TurretAtTarget", () -> turretAtTarget() ? 1.0 : 0.0));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(turret);
  }

  public boolean zeroTurret() {
    boolean didZero = false;
    double stringPotPosition = turret.getSensorCollection().getAnalogInRaw();
    if (stringPotPosition <= Constants.TurretConstants.kMaxStringPotZero
        && stringPotPosition >= Constants.TurretConstants.kMinStringPotZero) {
      int absPos = turret.getSensorCollection().getPulseWidthPosition() & 0xFFF;
      // inverted because absolute and relative encoders are out of phase
      int offset = (int) -(absPos - kTurretZeroTicks);
      turret.setSelectedSensorPosition(offset);
      didZero = true;
      logger.info(
          "Turret zeroed; offset: {} zeroTicks: {} absPosition: {}",
          offset,
          kTurretZeroTicks,
          absPos);
    } else {
      turret.configPeakOutputForward(0, 0);
      turret.configPeakOutputReverse(0, 0);
      logger.error("Turret zero failed. Killing turret...");
    }

    turret.clearStickyFaults();

    return didZero;
  }
}
