package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.kFastCruiseVelocity;
import static frc.robot.Constants.TurretConstants.kRotateByFinalKp;
import static frc.robot.Constants.TurretConstants.kSlowCruiseVelocity;
import static frc.robot.Constants.TurretConstants.kTurretId;
import static frc.robot.Constants.TurretConstants.kTurretTicksPerRadian;
import static frc.robot.Constants.TurretConstants.kTurretZeroTicks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private int turretStableCounts = 0;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private TurretState currentState = TurretState.IDLE;
  private final VisionSubsystem visionSubsystem;
  private MagazineSubsystem magazineSubsystem;
  private final DriveSubsystem driveSubsystem;
  private double targetTurretPosition = 0;
  private double cruiseVelocity = kFastCruiseVelocity;
  private boolean lastDoRotate = false;

  private int trackingStableCount;
  private int
      notValidTargetCount; // don't go to seeking if there is only a few frames of no valid target
  private int seekingCount = 0;
  private double rotateKp;

  public TurretSubsystem(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
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

  public boolean rotateBy(Rotation2d errorRotation2d) {
    double targetAngleRadians =
        errorRotation2d.getRadians()
            + turret.getSelectedSensorPosition() / kTurretTicksPerRadian // +
            + Math.toRadians(Constants.VisionConstants.kHorizAngleCorrectionDegrees);

    // if (Math.toDegrees(targetAngleRadians) > 360 || Math.toDegrees(targetAngleRadians) < 0) {
    //   rotateTo(getNonWrappedSetpoint(targetAngleRadians));
    //   return true;
    // }
    rotateTo(targetAngleRadians * -kTurretTicksPerRadian);
    return false;
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

  private double getNonWrappedSetpoint(double positionRadians) {
    positionRadians %= 2 * Math.PI;
    positionRadians = positionRadians < 0 ? positionRadians + 2 * Math.PI : positionRadians;

    return positionRadians * kTurretTicksPerRadian;
  }

  public void rotateTo(Rotation2d position) {
    double positionTicks = position.getRadians() * -kTurretTicksPerRadian;
    logger.info("Rotating Turret to {}", position);
    rotateTo(positionTicks);
  }

  public double getRotateByKp() {
    return rotateKp;
  }

  public void openLoopRotate(double percentOutput) {
    turret.set(ControlMode.PercentOutput, percentOutput);
  }

  // private
  public boolean isRotationFinished() {
    return trackingStableCount > TurretConstants.kRotateByStableCounts;
  }

  public boolean isTurretAtTarget() {
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
    return Set.of(
        new Measure("isRotationFinished", () -> isRotationFinished() ? 1.0 : 0.0),
        new Measure("TurretAtTarget", () -> isTurretAtTarget() ? 1.0 : 0.0),
        new Measure("rotateKp", this::getRotateByKp));
  }

  @Override
  public void registerWith(@NotNull TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(turret);
  }

  public void zeroTurret() {
    // double stringPotPosition = turret.getSensorCollection().getAnalogInRaw();
    if (!turret.getSensorCollection().isFwdLimitSwitchClosed()) {
      int absPos = turret.getSensorCollection().getPulseWidthPosition() & 0xFFF;
      // inverted because absolute and relative encoders are out of phase
      int offset = absPos - kTurretZeroTicks;
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
  // change back to private
  public void setSeekAngle(boolean seekLeft) {
    Pose2d pose = driveSubsystem.getPoseMeters();
    Translation2d deltaPosition = TurretConstants.kHubPositionMeters.minus(pose.getTranslation());
    Rotation2d seekingAngle = new Rotation2d(deltaPosition.getX(), deltaPosition.getY());
    logger.info("Pose: {}, Delta pose: {}, seek angle: {}", pose, deltaPosition, seekingAngle);
    seekingAngle = seekingAngle.minus(pose.getRotation());
    seekingAngle = seekingAngle.plus(TurretConstants.kTurretRobotOffset); // plus

    if (seekLeft) {
      seekingAngle = seekingAngle.minus(TurretConstants.kSeekAngleError); // plus
    } else {
      seekingAngle = seekingAngle.plus(TurretConstants.kSeekAngleError);
    }

    rotateTo(seekingAngle); // getNonWrappedSetpoint

    logger.info("Seeking angle is at: {}", seekingAngle);
  }

  private void setCruiseVelocity(boolean isFast) {
    if (isFast) {
      if (cruiseVelocity != kFastCruiseVelocity) {
        cruiseVelocity = kFastCruiseVelocity;
        turret.configMotionCruiseVelocity(cruiseVelocity);
      }
    } else {
      if (cruiseVelocity == kFastCruiseVelocity) {
        cruiseVelocity = kSlowCruiseVelocity;
        turret.configMotionCruiseVelocity(cruiseVelocity);
      }
    }
  }

  public void resetSeekCount() {
    seekingCount = 0;
  }

  public void trackTarget() {
    logger.info("Started tracking target");
    currentState = TurretState.SEEK_LEFT;
    seekingCount = 0;
    setSeekAngle(true);
  }

  public void stopTrackingTarget() {
    logger.info("switching to IDLE state");
    currentState = TurretState.IDLE;
  }

  public void fenderShot(boolean doRotate) {
    logger.info("{} -> FENDER_ADJUSTING}", currentState);
    currentState = TurretState.FENDER_ADJUSTING;
    lastDoRotate = doRotate;
    if (magazineSubsystem.isNextCargoAlliance() || (!doRotate)) {
      rotateTo(TurretConstants.kFenderAlliance);
    } else {
      rotateTo(TurretConstants.kFenderOpponent);
    }
  }

  public void fenderShot() {
    fenderShot(lastDoRotate);
  }

  @Override
  public void periodic() {
    HubTargetData targetData;
    Rotation2d errorRotation2d;
    switch (currentState) {
      case SEEK_LEFT:
        // fall through
      case SEEK_RIGHT:
        targetData = visionSubsystem.getTargetData();
        setCruiseVelocity(false);
        if (targetData.isValid()) {
          logger.info("{} -> AIMING", currentState);
          currentState = TurretState.AIMING;
          rotateTo(turret.getSelectedSensorPosition());
          // currentState = TurretState.IDLE;
          break;
        }
        if (isTurretAtTarget()) {
          seekingCount++;
          if (seekingCount > TurretConstants.kMaxSeekCount) {
            logger.info("{} -> IDLE", currentState);
            currentState = TurretState.IDLE;
            break;
          }
          if (currentState == TurretState.SEEK_LEFT) {
            setSeekAngle(false);
            logger.info("{} -> SEEK_RIGHT", currentState);
            currentState = TurretState.SEEK_RIGHT;
          } else {
            setSeekAngle(true);
            logger.info("{} -> LEFT", currentState);
            currentState = TurretState.SEEK_LEFT;
          }
        }
        break;
      case AIMING:
        // fall through
      case TRACKING:
        setCruiseVelocity(true);
        targetData = visionSubsystem.getTargetData();
        if (!targetData.isValid()) {
          notValidTargetCount++;

          if (notValidTargetCount > TurretConstants.kNotValidTargetCounts) {
            logger.info("{} -> SEEKING: {}", currentState, targetData);
            currentState = TurretState.SEEK_LEFT;
            seekingCount = 0;
            setSeekAngle(true);
            break;
          }
        } else {
          notValidTargetCount = 0;
        }
        errorRotation2d = targetData.getErrorRotation2d();

        if (Math.abs(errorRotation2d.getRadians())
            < TurretConstants.kCloseEnoughTarget.getRadians()) {
          trackingStableCount++;
        } else {
          trackingStableCount = 0;
        }

        TurretState nextState;
        if (trackingStableCount > TurretConstants.kRotateByStableCounts) {
          nextState = TurretState.TRACKING;
          rotateKp = kRotateByFinalKp;
          //          rotateKp = TurretConstants.kRotateByKp;
        } else {
          nextState = TurretState.AIMING;
          rotateKp = TurretConstants.kRotateByInitialKp;
        }
        if (rotateBy(errorRotation2d.times(rotateKp))) {
          logger.info("{} -> WRAPPING", currentState);
          currentState = TurretState.WRAPPING;
          break;
        }

        if (currentState != nextState) {
          logger.info("{} -> {}", currentState, nextState);
        }
        currentState = nextState;
        break;
      case WRAPPING:
        setCruiseVelocity(true);
        if (isTurretAtTarget()) {
          currentState = TurretState.AIMING;
          logger.info("WRAPPING-> AIMING");
        }
        break;
      case FENDER_ADJUSTING:
        setCruiseVelocity(true);
        if (isTurretAtTarget()) {
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
    SEEK_LEFT,
    SEEK_RIGHT,
    AIMING,
    TRACKING,
    IDLE,
    FENDER_ADJUSTING,
    FENDER_AIMED,
    WRAPPING;
  }
}
