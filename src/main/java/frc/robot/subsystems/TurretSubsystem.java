package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.kFastCruiseVelocity;
import static frc.robot.Constants.TurretConstants.kRotateByFinalKp;
import static frc.robot.Constants.TurretConstants.kSlowCruiseVelocity;
import static frc.robot.Constants.TurretConstants.kTurretId;
import static frc.robot.Constants.TurretConstants.kTurretTicksPerRadian;
import static frc.robot.Constants.TurretConstants.kTurretZeroTicks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
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
  private TalonFX turret;
  private int turretStableCounts = 0;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private TurretState currentState = TurretState.IDLE;
  private final VisionSubsystem visionSubsystem;
  private MagazineSubsystem magazineSubsystem;
  private final DriveSubsystem driveSubsystem;
  private double targetTurretPosition = 0;
  private double cruiseVelocity = kFastCruiseVelocity;
  private boolean lastDoRotate = false;
  private boolean isBallOne = true;
  private DigitalInput zeroTurretInput = new DigitalInput(9);

  private int trackingStableCount;
  private int wrappingCount = 0;
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
    turret = new TalonFX(kTurretId);
    turret.configAllSettings(Constants.TurretConstants.getTurretTalonConfig());
    turret.setNeutralMode(NeutralMode.Brake);
    turret.enableVoltageCompensation(true);
    turret.configSupplyCurrentLimit(Constants.TurretConstants.getSupplyCurrentLimitConfig());
    // turret.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10); // FIXME
    turret.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 10);
  }

  public List<BaseTalon> getTalons() {
    return List.of(turret);
  }

  public boolean visionRotateBy(Rotation2d errorRotation2d) {
    Rotation2d currentTurretPose =
        new Rotation2d(turret.getSelectedSensorPosition() / kTurretTicksPerRadian);
    Rotation2d horizontalAngleCorrection =
        Rotation2d.fromDegrees(VisionConstants.kHorizAngleCorrectionDegrees);

    // logger.info(
    //     "turret pose: {}, error: {}, horizontal correction: {}",
    //     currentTurretPose,
    //     errorRotation2d,
    //     horizontalAngleCorrection);
    Rotation2d targetAngle = currentTurretPose.minus(errorRotation2d);
    targetAngle = targetAngle.plus(horizontalAngleCorrection);
    targetAngle = targetAngle.plus(calcFeedForward());
    rotateTo(targetAngle);

    // double targetAngleRadians =
    //     errorRotation2d.getRadians()
    //         + turret.getSelectedSensorPosition() / kTurretTicksPerRadian // +
    //         + Math.toRadians(Constants.VisionConstants.kHorizAngleCorrectionDegrees);

    // if (Math.toDegrees(targetAngleRadians) > 360 || Math.toDegrees(targetAngleRadians) < 0) {
    //   rotateTo(getNonWrappedSetpoint(targetAngleRadians));
    //   return true;
    // }
    // rotateTo(targetAngleRadians * -kTurretTicksPerRadian);
    return false;
  }

  public Rotation2d calcFeedForward() {
    if (!driveSubsystem.getUseOdometry()) return new Rotation2d();
    double tangentVel = driveSubsystem.getTangentVelocity();
    double gyroRate = driveSubsystem.getGyroRate();
    Rotation2d feedForward = Rotation2d.fromDegrees(gyroRate * TurretConstants.kFYaw);
    feedForward =
        feedForward.plus(Rotation2d.fromDegrees(tangentVel * TurretConstants.kFTangentVelocity));
    return feedForward;
  }

  public void rotateBy(Rotation2d deltaRotation) {
    Rotation2d currentAngle =
        new Rotation2d(turret.getSelectedSensorPosition() / kTurretTicksPerRadian);
    Rotation2d targetAngle = currentAngle.plus(deltaRotation);
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
      // logger.info("Rotating Turret to {} ticks", setPointTicks);
    }
  }

  private double getNonWrappedSetpoint(double positionRadians) {
    positionRadians %= 2 * Math.PI;
    positionRadians = positionRadians < 0 ? positionRadians + 2 * Math.PI : positionRadians;

    return positionRadians * kTurretTicksPerRadian;
  }

  public void rotateTo(Rotation2d position) {
    double positionTicks = 0.0;
    if (Math.PI - Math.abs(position.getRadians()) <= TurretConstants.kOverlapAngle.getRadians()) {
      if (turret.getSelectedSensorPosition() < 0 && position.getRadians() > 0) {
        positionTicks =
            (-Math.PI - Math.abs(Rotation2d.fromDegrees(180).minus(position).getRadians()))
                * kTurretTicksPerRadian;
        logger.info(
            "Positive Overlap region POS: {}, turret pos: {}, PositionTicks: {}",
            position,
            turret.getSelectedSensorPosition(),
            positionTicks);
      } else if (turret.getSelectedSensorPosition() >= 0 && position.getRadians() < 0) {
        positionTicks =
            (Math.PI + position.plus(Rotation2d.fromDegrees(180)).getRadians())
                * kTurretTicksPerRadian;
        logger.info(
            "Negative Overlap region   POS: {}, turret pos: {}, PositionTicks: {}",
            position,
            turret.getSelectedSensorPosition(),
            positionTicks);
      } else {
        positionTicks = position.getRadians() * kTurretTicksPerRadian;
      }
    } else {
      positionTicks = position.getRadians() * kTurretTicksPerRadian;
    }
    // logger.info("Rotating Turret to {}", position);
    if (Math.abs(turret.getSelectedSensorPosition() - positionTicks) >= TurretConstants.kWrapTicks
        && (currentState == TurretState.AIMING || currentState == TurretState.TRACKING)) {
      logger.info("{} -> WRAPPING", currentState);
      currentState = TurretState.WRAPPING;
    }
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

  public boolean isTurretAtOdom() {
    double currentTurretPosition = turret.getSelectedSensorPosition();
    return Math.abs(targetTurretPosition - currentTurretPosition)
        > Constants.TurretConstants.kCloseEnoughTicks;
  }

  public Rotation2d getTurretRotation2d() {
    return new Rotation2d(
        turret.getSelectedSensorPosition() / TurretConstants.kTurretTicksPerRadian);
  }

  public double getTurretRadians() {
    return turret.getSelectedSensorPosition() / TurretConstants.kTurretTicksPerRadian;
  }

  public boolean isTurretAtZero() {
    return zeroTurretInput.get();
  }

  public void zeroTurret() {
    // double stringPotPosition = turret.getSensorCollection().getAnalogInRaw();
    if (zeroTurretInput.get()) {
      double absPos = turret.getSensorCollection().getIntegratedSensorAbsolutePosition();
      // inverted because absolute and relative encoders are out of phase
      double offset = absPos - kTurretZeroTicks;
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
  public void setSeekAngleLeft(boolean seekLeft) {
    double seekTicks = 0.0;
    // Pose2d pose = driveSubsystem.getPoseMeters();
    // Translation2d deltaPosition =
    // TurretConstants.kHubPositionMeters.minus(pose.getTranslation());
    // Rotation2d seekingAngle = new Rotation2d(deltaPosition.getX(), deltaPosition.getY());
    // logger.info("Pose: {}, Delta pose: {}, seek angle: {}", pose, deltaPosition, seekingAngle);
    // seekingAngle = seekingAngle.minus(pose.getRotation());
    // seekingAngle = seekingAngle.plus(TurretConstants.kTurretRobotOffset); // plus

    if (seekLeft) {
      seekTicks = TurretConstants.kForwardLimit;
      // seekingAngle = seekingAngle.minus(TurretConstants.kSeekAngleError); // plus
    } else {
      seekTicks = TurretConstants.kReverseLimit;
      // seekingAngle = seekingAngle.plus(TurretConstants.kSeekAngleError);
    }

    rotateTo(seekTicks); // getNonWrappedSetpoint

    logger.info("Seeking angle is at: {}", seekTicks);
  }

  public void seekCenter() {
    Pose2d pose = driveSubsystem.getPoseMeters();
    Translation2d deltaPosition = TurretConstants.kHubPositionMeters.minus(pose.getTranslation());
    Rotation2d seekAngle = new Rotation2d(deltaPosition.getX(), deltaPosition.getY());
    seekAngle = seekAngle.minus(pose.getRotation());
    seekAngle = seekAngle.plus(TurretConstants.kTurretRobotOffset);
    seekAngle = seekAngle.plus(calcFeedForward());
    logger.info(
        "Seek Center Angle: pose: {}, deltaPos: {}, seekAngle:{}", pose, deltaPosition, seekAngle);
    rotateTo(seekAngle);
  }

  public void inputOdomAim(Translation2d aimPosition) {
    Pose2d pose = driveSubsystem.getPoseMeters();
    Translation2d deltaPosition = aimPosition.minus(pose.getTranslation());
    Rotation2d findAngle = new Rotation2d(deltaPosition.getX(), deltaPosition.getY());
    findAngle = findAngle.minus(pose.getRotation());
    findAngle = findAngle.plus(TurretConstants.kTurretRobotOffset);
    double gyroRate = driveSubsystem.getGyroRate();
    Rotation2d feedForward = Rotation2d.fromDegrees(gyroRate * TurretConstants.kFYaw);
    findAngle = findAngle.plus(feedForward);
    rotateTo(findAngle);
  }

  // public void updateOpenLoopFeedFwd() {
  //   currentState = TurretState.IDLE;
  //   double[] driveVel = driveSubsystem.getDriveVelocity();
  //   double kF = TurretConstants.kFYaw;
  //   Rotation2d deltaRotation = new Rotation2d(driveSubsystem.gyroRate * kF);
  //   rotateBy(deltaRotation);
  // }

  private void setCruiseVelocityFast(boolean isFast) {
    if (isFast) {
      if (cruiseVelocity != kFastCruiseVelocity) {
        cruiseVelocity = kFastCruiseVelocity;
        logger.info("Set Cruise Vel Fast: {}", cruiseVelocity);
        turret.configMotionCruiseVelocity(cruiseVelocity);
        turret.configMotionAcceleration(TurretConstants.kFastAccel);
      }
    } else {
      if (cruiseVelocity == kFastCruiseVelocity) {
        cruiseVelocity = kSlowCruiseVelocity;
        logger.info("Set Cruise Vel Slow: {}", cruiseVelocity);
        turret.configMotionCruiseVelocity(cruiseVelocity);
        turret.configMotionAcceleration(TurretConstants.kSlowAccel);
      }
    }
  }

  public void resetSeekCount() {
    seekingCount = 0;
  }

  public void trackTarget() {
    logger.info("Started tracking target");
    // currentState = TurretState.SEEK_LEFT;
    // setSeekAngle(true);
    // HubTargetData targetData = visionSubsystem.getTargetData();
    setCruiseVelocityFast(true);
    if (visionSubsystem.isValid()) {
      currentState = TurretState.AIMING;
    } else {
      seekCenter();
      currentState = TurretState.SEEK_CENTER; // SEEK_CENTER
    }
  }

  public void trackOdom(Translation2d target) {
    inputOdomAim(target);
    currentState = TurretState.ODOM_FEED;
  }

  public void odometryAim() {
    logger.info("Tracking target with odometry");
    currentState = TurretState.ODOM_ADJUSTING;
    setCruiseVelocityFast(true);
    seekCenter();
  }

  public void stopTrackingTarget() {
    logger.info("switching to IDLE state");
    currentState = TurretState.IDLE;
  }

  public void fenderShot(boolean doRotate) {
    logger.info("{} -> FENDER_ADJUSTING", currentState);
    currentState = TurretState.FENDER_ADJUSTING;
    lastDoRotate = doRotate;
    if (magazineSubsystem.isNextCargoAlliance() || (!doRotate)) {
      rotateTo(TurretConstants.kFenderAlliance);
      logger.info(
          "Fender Shot: Turret 0deg, isOurs: {}, doRotate: {}",
          magazineSubsystem.isNextCargoAlliance(),
          doRotate);
    } else {
      rotateTo(TurretConstants.kFenderOpponent);
      logger.info("Fender Shot: Opponent Cargo");
    }
  }

  public void fenderShot() {
    fenderShot(lastDoRotate);
  }

  public void geyserShot(boolean isBallOne) {
    logger.info("{} -> GEYSER_ADJUSTING", currentState);
    currentState = TurretState.GEYSER_ADJUSTING;
    if (isBallOne) {
      rotateTo(TurretConstants.kGeyserBallOnePosition);
      logger.info("Geyser Shot: Starting Ball One Sequence");
    } else {
      rotateTo(TurretConstants.kGeyserBallTwoPosition);
      logger.info("Geyser Shot: Starting Ball Two Sequence");
    }
  }

  public void opponentCargoShot(Translation2d aimPosition) {
    logger.info("{} -> GEYSER_ADJUSTING", currentState);
    currentState = TurretState.GEYSER_ADJUSTING;
    inputOdomAim(aimPosition);
  }

  public void strykeShot(boolean isLeftClimb) {
    logger.info("{} -> STRYKE_ADJUSTING", currentState);
    currentState = TurretState.STRYKE_ADJUSTING;
    rotateTo(isLeftClimb ? TurretConstants.kOutsideStrykePos : TurretConstants.kInsideStrykePos);
  }

  @Override
  public void periodic() {
    HubTargetData targetData;
    Rotation2d errorRotation2d;
    switch (currentState) {
      case SEEK_CENTER:
        if (isTurretAtTarget()) {
          targetData = visionSubsystem.getTargetData();
          if (visionSubsystem.isValid()) {
            setCruiseVelocityFast(true);
            logger.info("SEEK_CENTER -> AIMING");
            logger.info("target data: {}", targetData);
            currentState = TurretState.AIMING;
            visionRotateBy(
                targetData.getErrorRotation2d().times(TurretConstants.kRotateByInitialKp));
          } else {
            setCruiseVelocityFast(false);
            logger.info("SEEK_CENTER -> SEEK_LEFT");
            currentState = TurretState.SEEK_LEFT;
            setSeekAngleLeft(true);
          }
        }
        break;
      case SEEK_LEFT:
        // fall through
      case SEEK_RIGHT:
        targetData = visionSubsystem.getTargetData();
        // setCruiseVelocityFast(false);
        if (visionSubsystem.isValid()) {
          logger.info("{} -> AIMING", currentState);
          logger.info("targetData: {}", targetData);
          setCruiseVelocityFast(true);
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
            setSeekAngleLeft(false);
            logger.info("{} -> SEEK_RIGHT", currentState);
            currentState = TurretState.SEEK_RIGHT;
          } else {
            setSeekAngleLeft(true);
            logger.info("{} -> LEFT", currentState);
            currentState = TurretState.SEEK_LEFT;
          }
        }
        break;
      case AIMING:
        // fall through
      case TRACKING:
        // setCruiseVelocityFast(false);
        targetData = visionSubsystem.getTargetData();
        if (!visionSubsystem.isValid()) {
          notValidTargetCount++;
          logger.info("notValidTargetCount: {}", notValidTargetCount);
          if (notValidTargetCount > TurretConstants.kNotValidTargetCounts) {
            logger.info("{} -> SEEK_CENTER: {}", currentState, targetData);
            currentState = TurretState.SEEK_CENTER;
            seekingCount = 0;
            seekCenter();
            break;
          }
          // If not valid but < not valid counts just hold last position
          break;
        } else {
          notValidTargetCount = 0;
        }
        if (visionSubsystem.isValid()) {
          errorRotation2d = targetData.getErrorRotation2d();
        } else {
          errorRotation2d = Rotation2d.fromDegrees(0);
        }

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
        if (currentState != nextState) {
          logger.info("{} -> {}", currentState, nextState);
        }
        currentState = nextState;
        if (visionRotateBy(errorRotation2d.times(rotateKp))) {
          logger.info("{} -> WRAPPING", currentState);
          currentState = TurretState.WRAPPING;
          setCruiseVelocityFast(true);
          break;
        }
        break;
      case WRAPPING:
        setCruiseVelocityFast(true);
        wrappingCount++;
        if (isTurretAtTarget() || wrappingCount > TurretConstants.kMaxWrapping) {
          currentState = TurretState.AIMING;
          wrappingCount = 0;
          logger.info("WRAPPING-> AIMING");
        } else {
          seekCenter();
        }
        break;
      case FENDER_ADJUSTING:
        setCruiseVelocityFast(true);
        if (isTurretAtTarget()) {
          currentState = TurretState.FENDER_AIMED;
          logger.info("FENDER_ADJUSTING -> FENDER_AIMED");
        }
        break;
      case FENDER_AIMED:
        // indicator for other subsystems
        break;
      case ODOM_FEED:
        // Will feed translation2ds every 20ms from magazine
        break;
      case ODOM_ADJUSTING:
        // setCruiseVelocityFast(true);
        if (isTurretAtTarget()) {
          currentState = TurretState.ODOM_AIMED;
          logger.info("ODOM_ADJUSTING -> ODOM_AIMED");
        }
        break;
      case ODOM_AIMED:
        // indicator for other subsystems
        break;
      case STRYKE_ADJUSTING:
        if (isTurretAtTarget()) {
          currentState = TurretState.STRYKE_AIMED;
          logger.info("STRYKE_ADJUSTING -> STRYKE_AIMED");
        }
        break;
      case STRYKE_AIMED:
        // indicator
        break;
      case GEYSER_ADJUSTING:
        if (isTurretAtTarget()) {
          currentState = TurretState.GEYSER_AIMED;
          logger.info("GEYSER_ADJUSTING -> GEYSER_AIMED");
        }
        break;
      case GEYSER_AIMED:
        // indicator for other subsystems
        break;
      case IDLE:
        // do nothing
        break;
      default:
        break;
    }
  }

  @Override
  public @NotNull Set<Measure> getMeasures() {
    return Set.of(
        new Measure("isRotationFinished", () -> isRotationFinished() ? 1.0 : 0.0),
        new Measure("TurretAtTarget", () -> isTurretAtTarget() ? 1.0 : 0.0),
        new Measure("rotateKp", this::getRotateByKp),
        new Measure("state", () -> currentState.ordinal()),
        new Measure("DIO", () -> zeroTurretInput.get() ? 1.0 : 0.0));
  }

  @Override
  public void registerWith(@NotNull TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(turret);
  }

  public enum TurretState {
    SEEK_CENTER,
    SEEK_LEFT,
    SEEK_RIGHT,
    AIMING,
    TRACKING,
    IDLE,
    FENDER_ADJUSTING,
    FENDER_AIMED,
    ODOM_ADJUSTING,
    ODOM_AIMED,
    STRYKE_ADJUSTING,
    STRYKE_AIMED,
    WRAPPING,
    GEYSER_ADJUSTING,
    GEYSER_AIMED,
    ODOM_FEED;
  }
}
