package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.kTurretId;
import static frc.robot.Constants.TurretConstants.kTurretMidpoint;
import static frc.robot.Constants.TurretConstants.kTurretTicksPerRadian;
import static frc.robot.Constants.TurretConstants.kTurretZeroTicks;
import static frc.robot.Constants.TurretConstants.kWrapRange;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import java.util.List;
import java.util.Set;
import org.jetbrains.annotations.NotNull;
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
  private final VisionSubsystem visionSubsystem;
  private MagazineSubsystem magazineSubsystem;

  private int trackingStableCount;

  public TurretSubsystem(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
    configTalons();
    zeroTurret();
  }

  public void setMagazineSubsystem(MagazineSubsystem magazineSubsystem) {
    this.magazineSubsystem = magazineSubsystem;
  }

  private void configTalons() {
    // turret setup
    turret = new TalonSRX(kTurretId);
    turret.configAllSettings(Constants.TurretConstants.getTurretTalonConfig());
    turret.setNeutralMode(NeutralMode.Brake);
    turret.enableCurrentLimit(false);
    turret.enableVoltageCompensation(true);
    turret.configSupplyCurrentLimit(Constants.TurretConstants.getSupplyCurrentLimitConfig());
    //    turret.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
  }

  public List<BaseTalon> getTalons() {
    return List.of(turret);
  }

  public void rotateBy(Rotation2d errorRotation2d) {

    Rotation2d targetAngle =
        new Rotation2d(turret.getSelectedSensorPosition() / kTurretTicksPerRadian); // current angle
    targetAngle = targetAngle.plus(errorRotation2d);
    targetAngle =
        targetAngle.plus(
            Rotation2d.fromDegrees(
                Constants.VisionConstants.kHorizAngleCorrectionDegrees)); // target angle
    if (targetAngle.getDegrees() <= kWrapRange
            && turret.getSelectedSensorPosition() > kTurretMidpoint
        || targetAngle.getDegrees() < 0) {
      targetAngle = targetAngle.plus(Rotation2d.fromDegrees(360)); // add 360 deg
    }
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
        > Constants.TurretConstants.kCloseEnoughTicks) {
      turretStableCounts = 0;
    } else {
      turretStableCounts++;
    }
    return turretStableCounts >= Constants.ShooterConstants.kStableCounts;
  }

  @Override
  public @NotNull Set<Measure> getMeasures() {
    return Set.of(new Measure("TurretAtTarget", () -> isRotationFinished() ? 1.0 : 0.0));
  }

  @Override
  public void registerWith(@NotNull TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(turret);
  }

  public void zeroTurret() {
    double stringPotPosition = turret.getSensorCollection().getAnalogInRaw();
    if (stringPotPosition <= Constants.TurretConstants.kMaxStringPotZero
        && stringPotPosition >= Constants.TurretConstants.kMinStringPotZero) {
      int absPos = turret.getSensorCollection().getPulseWidthPosition() & 0xFFF;
      // inverted because absolute and relative encoders are out of phase
      int offset = -(absPos - kTurretZeroTicks);
      turret.setSelectedSensorPosition(offset);
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
  }

  public TurretState getState() {
    return currentState;
  }

  public void trackTarget() {
    logger.info("Started tracking target");
    currentState = TurretState.SEEKING;
  }

  public void stopTrackingTarget() {
    logger.info("switching to IDLE state");
    currentState = TurretState.IDLE;
  }

  public void fenderShot() {
    logger.info("{} -> FENDER_ADJUSTING}", currentState);
    currentState = TurretState.FENDER_ADJUSTING;
    if (magazineSubsystem.isNextCargoAlliance()) {
      rotateTo(TurretConstants.kFenderAlliance);
    } else {
      rotateTo(TurretConstants.kFenderOpponent);
    }
  }

  @Override
  public void periodic() {
    HubTargetData targetData;
    Rotation2d errorRotation2d;
    double rotateKp;
    switch (currentState) {
      case SEEKING:
        // FIXME implement seek state
        // fall through
      case AIMING:
        // fall through
      case TRACKING:
        targetData = visionSubsystem.getTargetData();
        if (!targetData.isValid()) {
          logger.info("{} -> SEEKING: {}", currentState, targetData);
          currentState = TurretState.SEEKING;
          break;
        }
        errorRotation2d = targetData.getErrorRotation2d();
        rotateBy(errorRotation2d.times(TurretConstants.kRotateByKp));

        if (Math.abs(errorRotation2d.getRadians())
            < TurretConstants.kCloseEnoughTarget.getRadians()) {
          trackingStableCount++;
        } else {
          trackingStableCount = 0;
        }

        TurretState nextState;
        if (trackingStableCount > TurretConstants.kRotateByStableCounts) {
          nextState = TurretState.TRACKING;
          rotateKp = 0.95;
          //          rotateKp = TurretConstants.kRotateByKp;
        } else {
          nextState = TurretState.AIMING;
          rotateKp = TurretConstants.kRotateByKp;
        }

        if (currentState != nextState) {
          logger.info("{} -> {}", currentState, nextState);
        }
        currentState = nextState;
        break;
      case FENDER_ADJUSTING:
        if (isRotationFinished()) {
          currentState = TurretState.FENDER_AIMED;
          logger.info("FENDER_ADJUSTING -> FENDER_AIMED");
        }
        break;
      case FENDER_AIMED:
        // indicator for other subsystems
        break;
      case IDLE:
        // do nothing
        break;
      default:
        break;
    }
  }

  public enum TurretState {
    SEEKING,
    AIMING,
    TRACKING,
    IDLE,
    FENDER_ADJUSTING,
    FENDER_AIMED;
  }
}
