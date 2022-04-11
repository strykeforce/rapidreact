package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.MagazineConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TurretSubsystem.TurretState;
import java.util.List;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class MagazineSubsystem extends MeasurableSubsystem {
  private static final Logger logger = LoggerFactory.getLogger(MagazineSubsystem.class);
  private ColorSensorV3 colorSensor;
  private Color lastColor = new Color(0, 0, 0);
  private int lastProximity = 0;
  private TalonFX lowerMagazineFalcon;
  private TalonSRX upperMagazineTalon;
  private CargoColor[] storedCargoColors = new CargoColor[] {CargoColor.NONE, CargoColor.NONE};
  private CargoColor allianceCargoColor = CargoColor.NONE;
  private ColorMatch colorMatch = new ColorMatch();
  private LowerMagazineState currLowerMagazineState = LowerMagazineState.STOP;
  private UpperMagazineState currUpperMagazineState = UpperMagazineState.STOP;
  private final TurretSubsystem turretSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private Timer shootTimer = new Timer();
  private Timer ejectTimer = new Timer();
  private Timer readTimer = new Timer();
  private Timer timedShootTimer = new Timer();
  private boolean ignoreColorSensor = false;
  private boolean continueToShoot = false;
  private boolean isBeamBreakEnabled = false;
  private int shootUpperBeamStableCounts = 0;
  private boolean doTimedShoot = false;

  public MagazineSubsystem(
      TurretSubsystem turretSubsystem,
      VisionSubsystem visionSubsystem,
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    colorSensor = new ColorSensorV3(Port.kMXP);

    lowerMagazineFalcon = new TalonFX(MagazineConstants.kLowerMagazineTalonID);
    lowerMagazineFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    lowerMagazineFalcon.configAllSettings(
        MagazineConstants.getLowerMagazineTalonConfig(), Constants.kTalonConfigTimeout);
    lowerMagazineFalcon.configSupplyCurrentLimit(
        MagazineConstants.getLowerMagazineCurrentLimit(), Constants.kTalonConfigTimeout);
    lowerMagazineFalcon.enableVoltageCompensation(true);
    lowerMagazineFalcon.setNeutralMode(NeutralMode.Brake);

    upperMagazineTalon = new TalonSRX(MagazineConstants.kUpperMagazineTalonID);
    upperMagazineTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    upperMagazineTalon.configAllSettings(
        MagazineConstants.getUpperMagazineTalonConfig(), Constants.kTalonConfigTimeout);
    upperMagazineTalon.configSupplyCurrentLimit(
        MagazineConstants.getUpperMagazineCurrentLimit(), Constants.kTalonConfigTimeout);
    upperMagazineTalon.enableVoltageCompensation(true);
    upperMagazineTalon.setNeutralMode(NeutralMode.Brake);
    enableUpperBeamBreak(true);

    colorMatch.addColorMatch(MagazineConstants.kBlueCargo);
    colorMatch.addColorMatch(MagazineConstants.kRedCargo);
    colorMatch.addColorMatch(MagazineConstants.kNoCargo);
  }

  public void setShooterSubsystem(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
  }

  public void enableUpperBeamBreak(boolean enableUpper) {
    if (isBeamBreakEnabled != enableUpper) {
      logger.info("Enabling upper talon limit switch: {}", enableUpper);
    }
    if (enableUpper) {
      upperMagazineTalon.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
      isBeamBreakEnabled = true;
    } else {
      upperMagazineTalon.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
      isBeamBreakEnabled = false;
    }
  }

  public void enableLowerBeamBreak(boolean enableLower) {
    if (isBeamBreakEnabled != enableLower) {
      logger.info("Enabling lower talon limit switch: {}", enableLower);
    }
    if (enableLower) {
      lowerMagazineFalcon.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
      isBeamBreakEnabled = true;
    } else {
      lowerMagazineFalcon.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
      isBeamBreakEnabled = false;
    }
  }

  public void lowerOpenLoopRotate(double percentOutput) {
    lowerMagazineFalcon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Lower magazine motor turned on {}", percentOutput);
  }

  public void upperOpenLoopRotate(double percentOutput) {
    upperMagazineTalon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Upper magazine motor turned on {}", percentOutput);
  }

  public List<BaseTalon> getTalons() {
    return List.of(lowerMagazineFalcon, upperMagazineTalon);
  }

  public void lowerClosedLoopRotate(double speed) {
    lowerMagazineFalcon.set(ControlMode.Velocity, speed);
    logger.info("Lower magazine motor closedLoopRotate {}", speed);
  }

  public void upperClosedLoopRotate(double speed) {
    upperMagazineTalon.set(ControlMode.Velocity, speed);
    logger.info("Upper magazine motor closedLoopRotate {}", speed);
  }

  public void stopMagazine() {
    logger.info("Stopping Magazine");
    currLowerMagazineState = LowerMagazineState.STOP;
    currUpperMagazineState = UpperMagazineState.STOP;
    lowerMagazineFalcon.set(ControlMode.PercentOutput, 0.0);
    if (storedCargoColors[0] != CargoColor.NONE)
      upperMagazineTalon.set(ControlMode.Velocity, MagazineConstants.kUpperMagazineIntakeSpeed);
    else upperClosedLoopRotate(0.0);
  }

  public boolean isLowerBeamBroken() {
    boolean lowerBeam = (lowerMagazineFalcon.getSensorCollection().isFwdLimitSwitchClosed() == 1);
    return lowerBeam;
  }

  public boolean isUpperBeamBroken() {
    boolean upperBeam = upperMagazineTalon.getSensorCollection().isFwdLimitSwitchClosed();
    return upperBeam;
  }

  public Color getColor() {
    lastColor = colorSensor.getColor();
    if (lastColor.red == 0 && lastColor.green == 0 && lastColor.blue == 0) {
      ignoreColorSensor = true;
      logger.warn("Color sensor error. Diasbling color sensor.");
      if (allianceCargoColor == CargoColor.BLUE) lastColor = MagazineConstants.kBlueCargo;
      else lastColor = MagazineConstants.kRedCargo;
    }
    // lastColor = new Color(0, 0, 0);
    return lastColor;
  }

  public int getProximity() {
    lastProximity = colorSensor.getProximity();
    return lastProximity;
  }

  public void setAllianceColor(Alliance alliance) {
    allianceCargoColor = alliance == Alliance.Red ? CargoColor.RED : CargoColor.BLUE;
  }

  public void ignoreColorSensor(boolean ignore) {
    ignoreColorSensor = ignore;
    logger.info("set ignoreColorSensor to: {}", ignore);
  }

  public boolean isColorSensorIgnored() {
    return ignoreColorSensor;
  }

  public boolean isNextCargoAlliance() {
    return ignoreColorSensor
        || storedCargoColors[0] == allianceCargoColor
        || storedCargoColors[0] == CargoColor.NONE;
  }

  public boolean isFirstCargoAlliance() {
    return (!isMagazineFull()
            && (storedCargoColors[0] == allianceCargoColor
                || storedCargoColors[1] == allianceCargoColor))
        || (isMagazineFull() && storedCargoColors[0] == allianceCargoColor);
  }

  public CargoColor readCargoColor() {
    CargoColor currentCargoColor = CargoColor.NONE;
    if (!ignoreColorSensor) {
      Color readColor = getColor();
      ColorMatchResult matchResult = colorMatch.matchClosestColor(readColor);

      if (matchResult.color == MagazineConstants.kBlueCargo) {
        currentCargoColor = CargoColor.BLUE;
      } else if (matchResult.color == MagazineConstants.kRedCargo) {
        currentCargoColor = CargoColor.RED;
      }

      if (currentCargoColor == CargoColor.NONE) return currentCargoColor;
    } else {
      // Ignoring color sensor results
      currentCargoColor = allianceCargoColor;
    }

    if (storedCargoColors[1] == CargoColor.NONE) {
      storedCargoColors[1] = currentCargoColor;
      logger.info("Added cargo {}", currentCargoColor);
    } else {
      logger.error("Picked up third cargo {}, not recording", currentCargoColor);
    }

    return currentCargoColor;
  }

  public void shotOneCargo() {
    logger.info("Shot {} cargo", storedCargoColors[0]);
    storedCargoColors[0] = CargoColor.NONE;
  }

  public CargoColor getNextCargo() {
    return storedCargoColors[0];
  }

  public CargoColor[] getAllCargoColors() {
    return storedCargoColors;
  }

  public void clearCargoColors() {
    storedCargoColors[0] = CargoColor.NONE;
    storedCargoColors[1] = CargoColor.NONE;
  }

  public void preloadCargo() {
    upperClosedLoopRotate(MagazineConstants.kUpperMagazineIndexSpeed);
    storedCargoColors[0] = allianceCargoColor;
    logger.info("Preloading {} Cargo", allianceCargoColor);
  }

  public void indexCargo() {
    continueToShoot = false;
    enableUpperBeamBreak(true);
    enableLowerBeamBreak(true);
    logger.info("Start indexing cargo");
    logger.info(
        "lower {} -> WAIT_CARGO, upper {} -> EMPTY",
        currLowerMagazineState,
        currUpperMagazineState);
    currLowerMagazineState = LowerMagazineState.WAIT_CARGO;
    currUpperMagazineState = UpperMagazineState.EMPTY;
    upperClosedLoopRotate(MagazineConstants.kUpperMagazineIntakeSpeed);
    lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIntakeSpeed);
  }

  public void manualLowerMagazine(double lowerSpeed) {
    if (lowerSpeed == 0.0) currLowerMagazineState = LowerMagazineState.STOP;
    else currLowerMagazineState = LowerMagazineState.MANUAL;
    lowerOpenLoopRotate(lowerSpeed);
  }

  public void manualUpperMagazine(double upperSpeed) {
    if (upperSpeed == 0.0) currUpperMagazineState = UpperMagazineState.STOP;
    else currUpperMagazineState = UpperMagazineState.MANUAL;
    upperOpenLoopRotate(upperSpeed);
  }

  public void manualClosedLoopFullMagazine(double lowerSpeed, double upperSpeed) {
    if (upperSpeed == 0.0 && lowerSpeed == 0.0) {
      currLowerMagazineState = LowerMagazineState.STOP;
      currUpperMagazineState = UpperMagazineState.STOP;
    } else {
      currLowerMagazineState = LowerMagazineState.MANUAL;
      currUpperMagazineState = UpperMagazineState.MANUAL;
    }

    lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIntakeSpeed);
    upperClosedLoopRotate(MagazineConstants.kUpperMagazineIndexSpeed);
  }

  public void manualEjectCargoReverse(double lowerSpeed, double upperSpeed) {
    enableUpperBeamBreak(false);
    lowerClosedLoopRotate(lowerSpeed);
    upperClosedLoopRotate(upperSpeed);
    clearCargoColors();
    currLowerMagazineState = LowerMagazineState.MANUAL;
    currUpperMagazineState = UpperMagazineState.MANUAL;
  }

  private void autoStopUpperMagazine(double speed) {
    if (isUpperBeamBroken() && upperMagazineTalon.getMotorOutputPercent() != 0.0) {
      lowerOpenLoopRotate(0.0);
      logger.info("Stopping upper magazine, upper beam broken");
    } else if (!isUpperBeamBroken() && upperMagazineTalon.getMotorOutputPercent() == 0.0) {
      upperClosedLoopRotate(speed);
      logger.info("Upper beam not broken, upper magazine running");
    }
  }

  public boolean isShootSequenceDone() {
    return isMagazineEmpty()
        && !isUpperBeamBroken()
        && getCurrUpperMagazineState() == UpperMagazineState.EMPTY;
  }

  public boolean isMagazineFull() {
    return storedCargoColors[0] != CargoColor.NONE && storedCargoColors[1] != CargoColor.NONE;
  }

  public boolean isMagazineEmpty() {
    return storedCargoColors[0] == CargoColor.NONE && storedCargoColors[1] == CargoColor.NONE;
  }

  public void magazineInterrupted() {
    currLowerMagazineState = LowerMagazineState.STOP;
    stopMagazine();
    logger.info("Magazine interrupted, switching lower state to stop");
  }

  public void shoot() {
    // if (storedCargoColors[0] == CargoColor.NONE && !ignoreColorSensor) {
    //   logger.info("Magazine empty, not shooting");
    //   currMagazineState = MagazineState.STOP;
    // } else {
    // logger.info("{} -> PAUSE}", currMagazineState);
    enableUpperBeamBreak(true);
    logger.info("Shooting Cargo");
    if (currLowerMagazineState == LowerMagazineState.STOP) {
      logger.info("lower {} -> WAIT_CARGO", currLowerMagazineState);
      enableLowerBeamBreak(true);
      lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIntakeSpeed);
    }
    if (currUpperMagazineState == UpperMagazineState.STOP) {
      logger.info("upper {} -> EMPTY", currUpperMagazineState);
      currUpperMagazineState = UpperMagazineState.EMPTY;
    }
    continueToShoot = true;
    doTimedShoot = false;
    // }
  }

  public void timedShoot() {
    doTimedShoot = true;
    continueToShoot = true;
    enableUpperBeamBreak(true);
    enableLowerBeamBreak(true);
    if (currLowerMagazineState == LowerMagazineState.STOP) {
      logger.info("lower {} -> WAIT_CARGO", currLowerMagazineState);
      lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIntakeSpeed);
    }
    if (currUpperMagazineState == UpperMagazineState.STOP) {
      logger.info("upper {} -> EMPTY", currUpperMagazineState);
      currUpperMagazineState = UpperMagazineState.EMPTY;
    }
  }

  public void setManualState() {
    currLowerMagazineState = LowerMagazineState.MANUAL;
    currUpperMagazineState = UpperMagazineState.MANUAL;
  }

  public LowerMagazineState getCurrLowerMagazineState() {
    return currLowerMagazineState;
  }

  public UpperMagazineState getCurrUpperMagazineState() {
    return currUpperMagazineState;
  }

  @Override
  public void periodic() {
    switch (currLowerMagazineState) {
      case MANUAL:
        break;

      case WAIT_CARGO:
        // Check number of cargo
        // enableUpperBeamBreak(true);
        if (isMagazineFull()) {
          logger.info("WAIT_CARGO -> WAIT_UPPER");
          currLowerMagazineState = LowerMagazineState.WAIT_UPPER;
          lowerOpenLoopRotate(0.0);
          break;
        } else if (storedCargoColors[1] == CargoColor.NONE) {
          // if (lowerMagazineFalcon.getMotorOutputPercent() == 0.0) {
          //   lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIntakeSpeed);
          // }
        }
        // Knowing when to read cargo color
        if (isLowerBeamBroken()) {
          logger.info("WAIT_CARGO -> READ_CARGO");
          currLowerMagazineState = LowerMagazineState.READ_CARGO;
          lowerOpenLoopRotate(0.0);
          readTimer.reset();
          readTimer.start();
        }
        break;

      case READ_CARGO:
        // Read cargo color, switch states depending on amount of cargo
        CargoColor cargoColor = readCargoColor();
        boolean hasReadElapsed = readTimer.hasElapsed(MagazineConstants.kReadTimerDelay);
        if (cargoColor != CargoColor.NONE || hasReadElapsed) {
          // ignoreColorSensor || storedCargoColors[0]
          if (hasReadElapsed) {
            logger.info("ReadTimer Elapsed");
            cargoColor = allianceCargoColor;
            storedCargoColors[1] = cargoColor;
          }
          if (cargoColor != allianceCargoColor && !ignoreColorSensor) {
            logger.info("READ_CARGO -> EJECT_CARGO");
            lowerClosedLoopRotate(MagazineConstants.kLowerMagazineEjectSpeed);
            currLowerMagazineState = LowerMagazineState.EJECT_CARGO;
            ejectTimer.reset();
            ejectTimer.start();
            break;
          } else if (currUpperMagazineState == UpperMagazineState.EMPTY) {
            logger.info("READ_CARGO -> WAIT_EMPTY");
            currLowerMagazineState = LowerMagazineState.WAIT_EMPTY;
            enableLowerBeamBreak(false);
            lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIndexSpeed);
            upperClosedLoopRotate(MagazineConstants.kUpperMagazineIndexSpeed);
            break;
          } else {
            logger.info("READ_CARGO -> WAIT_UPPER");
            currLowerMagazineState = LowerMagazineState.WAIT_UPPER;
            break;
          }
        }
        break;

      case EJECT_CARGO:
        if (ejectTimer.hasElapsed(MagazineConstants.kEjectTimerDelay)) {
          logger.info("EJECT_CARGO -> WAIT_CARGO");
          storedCargoColors[1] = CargoColor.NONE;
          currLowerMagazineState = LowerMagazineState.WAIT_CARGO;
          lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIntakeSpeed);
        }
        break;

      case WAIT_UPPER:
        if (currUpperMagazineState == UpperMagazineState.EMPTY
            || currUpperMagazineState == UpperMagazineState.CARGO_SHOT && !isUpperBeamBroken()) {
          logger.info("WAIT_UPPER -> WAIT_EMPTY");
          enableLowerBeamBreak(false);
          lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIndexSpeed);
          upperClosedLoopRotate(MagazineConstants.kUpperMagazineIndexSpeed);
          currLowerMagazineState = LowerMagazineState.WAIT_EMPTY;
        }
        break;

      case WAIT_EMPTY:
        if (!isLowerBeamBroken()) {
          logger.info("WAIT_EMPTY -> WAIT_CARGO");
          storedCargoColors[0] = storedCargoColors[1];
          storedCargoColors[1] = CargoColor.NONE;
          enableLowerBeamBreak(true);
          currLowerMagazineState = LowerMagazineState.WAIT_CARGO;
        }
        break;

      case TIMED_FEED:
        if (timedShootTimer.hasElapsed(MagazineConstants.kTimedShootTimerDelay)) {
          logger.info("TIMED_FEED -> WAIT_CARGO");
          currLowerMagazineState = LowerMagazineState.WAIT_CARGO;
          enableLowerBeamBreak(true);
        }
        break;
      case STOP:
        break;
    }

    switch (currUpperMagazineState) {
      case MANUAL:
        break;

      case SHOOT:
        if (!isUpperBeamBroken()) shootUpperBeamStableCounts++;
        else shootUpperBeamStableCounts = 0;

        if (shootUpperBeamStableCounts > MagazineConstants.kShootUpperBeamStableCounts) {
          logger.info("SHOOT -> CARGO_SHOT");
          currUpperMagazineState = UpperMagazineState.CARGO_SHOT;
          shootTimer.reset();
          shootTimer.start();
          enableUpperBeamBreak(true);
          upperClosedLoopRotate(MagazineConstants.kUpperMagazineIndexSpeed);
          shotOneCargo();
        }
        break;

      case CARGO_SHOT:
        if (shootTimer.hasElapsed(MagazineConstants.kShootDelay)) {
          logger.info("CARGO_SHOT -> EMPTY");
          currUpperMagazineState = UpperMagazineState.EMPTY;
          if (turretSubsystem.getState() == TurretState.GEYSER_AIMED) {
            turretSubsystem.geyserShot(false);
          }
          if (isMagazineEmpty()) upperClosedLoopRotate(0.0);
          break;
        }
        break;

      case EMPTY:
        if (isUpperBeamBroken()) {
          logger.info("EMPTY -> WAIT_AIM");
          currUpperMagazineState = UpperMagazineState.WAIT_AIM;
          upperClosedLoopRotate(MagazineConstants.kUpperMagazineIndexSpeed);
        }
        break;

      case WAIT_AIM:
        if (continueToShoot) {
          if (turretSubsystem.getState() == TurretState.FENDER_AIMED) {
            logger.info("WAIT_AIM -> PAUSE");
            shooterSubsystem.fenderShot();
            turretSubsystem.fenderShot();
            currUpperMagazineState = UpperMagazineState.PAUSE;
          } else if (turretSubsystem.getState() == TurretState.TRACKING
              && visionSubsystem.isValid()) {
            logger.info("WAIT_AIM -> PAUSE");
            shooterSubsystem.shoot();
            currUpperMagazineState = UpperMagazineState.PAUSE;
          } else if (turretSubsystem.getState() == TurretState.ODOM_AIMED
              || turretSubsystem.getState() == TurretState.GEYSER_AIMED) {
            logger.info("WAIT_AIM -> PAUSE");
            currUpperMagazineState = UpperMagazineState.PAUSE;
          }
        }
        break;

      case PAUSE:
        // Vision Shoot
        if (shooterSubsystem.getCurrentState() == ShooterState.SHOOT
            && turretSubsystem.getState() == TurretState.TRACKING
            && visionSubsystem.isPixelWidthStable()
            && driveSubsystem.isVelocityStable()) {
          shooterSubsystem.logShotSol();
          if (!shooterSubsystem.isLastLookupBeyondTable()) {
            Pose2d calcPose =
                visionSubsystem.getVisionOdometry(
                    turretSubsystem.getTurretRotation2d(),
                    driveSubsystem.getGyroRotation2d(),
                    shooterSubsystem.getLastLookupDistance());
            driveSubsystem.updateOdometryWithVision(calcPose);
          }
          if (doTimedShoot) {
            logger.info("PAUSE -> TIMED_FEED");
            currUpperMagazineState = UpperMagazineState.TIMED_FEED;
            currLowerMagazineState = LowerMagazineState.TIMED_FEED;
            enableUpperBeamBreak(false);
            enableLowerBeamBreak(false);
            upperClosedLoopRotate(MagazineConstants.kUpperMagazineFeedSpeed);
            lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIndexSpeed);
            timedShootTimer.reset();
            timedShootTimer.start();
          } else {
            logger.info("PAUSE -> SHOOT");
            enableUpperBeamBreak(false);
            upperClosedLoopRotate(MagazineConstants.kUpperMagazineFeedSpeed);
            currUpperMagazineState = UpperMagazineState.SHOOT;
          }
          // Fender Shot || Odometry Aim
        } else if (shooterSubsystem.getCurrentState() == ShooterState.SHOOT
            && (turretSubsystem.getState() == TurretState.FENDER_AIMED
                || turretSubsystem.getState() == TurretState.ODOM_AIMED
                || turretSubsystem.getState() == TurretState.GEYSER_AIMED)) {
          logger.info("PAUSE -> SHOOT");
          enableUpperBeamBreak(false);
          upperClosedLoopRotate(MagazineConstants.kUpperMagazineFeedSpeed);
          currUpperMagazineState = UpperMagazineState.SHOOT;
        } else if (turretSubsystem.getState() == TurretState.TRACKING
            && visionSubsystem.isValid()) {
          shooterSubsystem.shoot();
        }
        break;

      case TIMED_FEED:
        if (timedShootTimer.hasElapsed(MagazineConstants.kTimedShootTimerDelay)) {
          logger.info("TIMED_FEED -> EMPTY");
          currUpperMagazineState = UpperMagazineState.EMPTY;
          clearCargoColors();
          enableUpperBeamBreak(true);
        }
        break;

      case STOP:
        break;
    }
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(lowerMagazineFalcon);
    telemetryService.register(upperMagazineTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("red", () -> lastColor.red),
        new Measure("blue", () -> lastColor.blue),
        new Measure("green", () -> lastColor.green),
        new Measure("Proximity", () -> lastProximity),
        new Measure("Lower Mag State", () -> currLowerMagazineState.ordinal()),
        new Measure("Upper Mag State", () -> currUpperMagazineState.ordinal()));
  }

  public enum CargoColor {
    RED("red"),
    BLUE("blue"),
    NONE("black");

    public String color;

    private CargoColor(String color) {
      this.color = color;
    }
  }

  public enum LowerMagazineState {
    MANUAL,
    WAIT_CARGO,
    READ_CARGO,
    WAIT_UPPER,
    WAIT_EMPTY,
    EJECT_CARGO,
    TIMED_FEED,
    STOP;
  }

  public enum UpperMagazineState {
    MANUAL,
    EMPTY,
    WAIT_AIM,
    SHOOT,
    CARGO_SHOT,
    PAUSE,
    TIMED_FEED,
    STOP;
  }
}
