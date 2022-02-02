package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.kTurretId;
import static frc.robot.Constants.TurretConstants.kTurretMidpoint;
import static frc.robot.Constants.TurretConstants.kTurretTicksPerRadian;
import static frc.robot.Constants.TurretConstants.kTurretZeroTicks;
import static frc.robot.Constants.TurretConstants.kWrapRange;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import java.util.List;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class TurretSubsystem extends MeasurableSubsystem {

  public boolean talonReset;
  private TalonSRX turret;
  private double targetTurretPosition = 0;
  private int turretStableCounts = 0;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private TurretState currentState = TurretState.IDLE;

  public TurretSubsystem() {
    configTalons();
  }

  private void configTalons() {
    // turret setup
    turret = new TalonSRX(kTurretId);
    turret.configAllSettings(Constants.TurretConstants.getTurretTalonConfig());
    turret.setNeutralMode(NeutralMode.Brake);
    turret.enableCurrentLimit(false);
    turret.enableVoltageCompensation(true);
    turret.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5, 30, 500));
  }

  public List<BaseTalon> getTalons() {
    return List.of(turret);
  }

  public void rotateTurret(Rotation2d errorRotation2d) {
    Rotation2d targetAngle =
        new Rotation2d(turret.getSelectedSensorPosition() / kTurretTicksPerRadian); // current angle
    targetAngle
        .plus(errorRotation2d)
        .plus(
            Rotation2d.fromDegrees(
                Constants.VisionConstants.kHorizAngleCorrection)); // target angle
    if (targetAngle.getDegrees() <= kWrapRange
            && turret.getSelectedSensorPosition() > kTurretMidpoint
        || targetAngle.getDegrees() < 0) {
      targetAngle.plus(Rotation2d.fromDegrees(360)); // add 360 deg
    }
    targetAngle.times(kTurretTicksPerRadian); // setpoint
    rotateTo(targetAngle);
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(turret.getSelectedSensorPosition() / kTurretTicksPerRadian);
  }

  public void rotateTo(double setPointTicks) {
    if (setPointTicks < TurretConstants.kReverseLimit) {
      setPointTicks = TurretConstants.kReverseLimit;
    }

    if (setPointTicks > TurretConstants.kForwardLimit) {
      setPointTicks = TurretConstants.kForwardLimit;
    }

    if (turret.hasResetOccurred()) {
      talonReset = true;
    } else {
      targetTurretPosition = setPointTicks;
      turret.set(ControlMode.MotionMagic, setPointTicks);
      logger.info("Rotating Turret to {} ticks", setPointTicks);
    }
  }

  public void rotateTo(Rotation2d position) {
    double positionTicks = position.getRadians() * kTurretTicksPerRadian;
    logger.info("Rotating Turret to {}", position);
    rotateTo(positionTicks);
  }

  public void openLoopRotate(double percentOutput) {
    turret.set(ControlMode.PercentOutput, percentOutput);
  }

  public boolean isRotationFinished() {
    double currentTurretPosition = turret.getSelectedSensorPosition();
    if (Math.abs(targetTurretPosition - currentTurretPosition)
        > Constants.TurretConstants.kCloseEnoughTurret) {
      turretStableCounts = 0;
    } else {
      turretStableCounts++;
    }
    return turretStableCounts >= Constants.ShooterConstants.kStableCounts;
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("TurretAtTarget", () -> isRotationFinished() ? 1.0 : 0.0));
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

  @Override
  public void periodic() {
    switch (currentState) {
      case AIMING:
        break;
      case AIMING_DONE:
        break;
      case IDLE:
        break;
      default:
        break;
    }
  }

  public enum TurretState {
    AIMING,
    AIMING_DONE,
    IDLE
  }
}
