package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.MagazineConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TurretSubsystem.TurretState;
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
  private TalonSRX lowerMagazineTalon;
  private TalonSRX upperMagazineTalon;
  private CargoColor[] storedCargoColors = new CargoColor[] {CargoColor.NONE, CargoColor.NONE};
  private CargoColor allianceCargoColor = CargoColor.NONE;
  private ColorMatch colorMatch = new ColorMatch();
  private MagazineState currMagazineState = MagazineState.STOP;
  private final TurretSubsystem turretSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private Timer shootTimer = new Timer();
  private Timer ejectTimer = new Timer();
  private boolean ignoreColorSensor = false;
  private int shootUpperBeamStableCounts = 0;

  public MagazineSubsystem(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    colorSensor = new ColorSensorV3(Port.kMXP);

    lowerMagazineTalon = new TalonSRX(MagazineConstants.kLowerMagazineTalonID);
    lowerMagazineTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    lowerMagazineTalon.configAllSettings(
        MagazineConstants.getLowerMagazineTalonConfig(), Constants.kTalonConfigTimeout);
    lowerMagazineTalon.configSupplyCurrentLimit(
        MagazineConstants.getLowerMagazineCurrentLimit(), Constants.kTalonConfigTimeout);
    lowerMagazineTalon.enableVoltageCompensation(true);
    lowerMagazineTalon.setNeutralMode(NeutralMode.Brake);

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
    upperClosedLoopRotate(MagazineConstants.kUpperMagazineIntakeSpeed);
  }

  public void setShooterSubsystem(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
  }

  public void enableUpperBeamBreak(boolean enable) {
    if (enable) {
      upperMagazineTalon.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    } else {
      upperMagazineTalon.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
    }
    logger.info("Enabling talon limit switch: {}", enable);
  }

  public void lowerOpenLoopRotate(double percentOutput) {
    lowerMagazineTalon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Lower magazine motor turned on {}", percentOutput);
  }

  public void upperOpenLoopRotate(double percentOutput) {
    upperMagazineTalon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Upper magazine motor turned on {}", percentOutput);
  }

  public void lowerClosedLoopRotate(double speed) {
    lowerMagazineTalon.set(ControlMode.Velocity, speed);
    logger.info("Lower magazine motor closedLoopRotate {}", speed);
  }

  public void upperClosedLoopRotate(double speed) {
    upperMagazineTalon.set(ControlMode.Velocity, speed);
    logger.info("Upper magazine motor closedLoopRotate {}", speed);
  }

  public void stopMagazine() {
    logger.info("Stopping Magazine");
    currMagazineState = MagazineState.STOP;
    lowerMagazineTalon.set(ControlMode.PercentOutput, 0.0);
    upperMagazineTalon.set(ControlMode.Velocity, MagazineConstants.kUpperMagazineIntakeSpeed);
  }

  public boolean isLowerBeamBroken() {
    boolean lowerBeam = lowerMagazineTalon.getSensorCollection().isFwdLimitSwitchClosed();
    return lowerBeam;
  }

  public boolean isUpperBeamBroken() {
    boolean upperBeam = upperMagazineTalon.getSensorCollection().isFwdLimitSwitchClosed();
    return upperBeam;
  }

  public Color getColor() {
    lastColor = colorSensor.getColor();
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

    if (storedCargoColors[0] == CargoColor.NONE) {
      storedCargoColors[0] = currentCargoColor;
      logger.info("Added first cargo {}", currentCargoColor);
    } else if (storedCargoColors[1] == CargoColor.NONE) {
      storedCargoColors[1] = currentCargoColor;
      logger.info("Added second cargo {}", currentCargoColor);
    } else {
      logger.error("Picked up third cargo {}, not recording", currentCargoColor);
    }

    return currentCargoColor;
  }

  public void shotOneCargo() {
    logger.info("Shot {} cargo", storedCargoColors[0]);
    storedCargoColors[0] = storedCargoColors[1];
    storedCargoColors[1] = CargoColor.NONE;
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
    storedCargoColors[0] = allianceCargoColor;
    logger.info("Preloading {} Cargo", allianceCargoColor);
  }

  public void indexCargo() {
    enableUpperBeamBreak(true);
    logger.info("Start indexing cargo");
    currMagazineState = MagazineState.WAIT_CARGO;
  }

  public void manualLowerMagazine(double lowerSpeed) {
    if (lowerSpeed == 0.0) currMagazineState = MagazineState.STOP;
    else currMagazineState = MagazineState.MANUAL_INTAKE;
    lowerOpenLoopRotate(lowerSpeed);
  }

  public void manualUpperMagazine(double upperSpeed) {
    if (upperSpeed == 0.0) currMagazineState = MagazineState.STOP;
    else currMagazineState = MagazineState.MANUAL_INTAKE;
    upperOpenLoopRotate(upperSpeed);
  }

  public void manualClosedLoopFullMagazine(double lowerSpeed, double upperSpeed) {
    if (upperSpeed == 0.0 && lowerSpeed == 0.0) currMagazineState = MagazineState.STOP;
    else currMagazineState = MagazineState.MANUAL_INTAKE;

    lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIntakeSpeed);
    upperClosedLoopRotate(MagazineConstants.kUpperMagazineIndexSpeed);
  }

  public void manualEjectCargoReverse(double lowerSpeed, double upperSpeed) {
    enableUpperBeamBreak(false);
    lowerClosedLoopRotate(lowerSpeed);
    upperClosedLoopRotate(upperSpeed);
    clearCargoColors();
    currMagazineState = MagazineState.MANUAL_INTAKE;
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

  public boolean isMagazineFull() {
    return currMagazineState == MagazineState.STOP;
  }

  public void magazineInterrupted() {
    currMagazineState = MagazineState.STOP;
    logger.info("Magazine interrupted, switching state to stop");
  }

  public void shoot() {
    // if (storedCargoColors[0] == CargoColor.NONE && !ignoreColorSensor) {
    //   logger.info("Magazine empty, not shooting");
    //   currMagazineState = MagazineState.STOP;
    // } else {
    logger.info("{} -> PAUSE}", currMagazineState);
    currMagazineState = MagazineState.PAUSE;
    // }
  }

  public MagazineState getCurrMagazineState() {
    return currMagazineState;
  }

  @Override
  public void periodic() {
    switch (currMagazineState) {
      case MANUAL_INTAKE:
        break;

      case WAIT_CARGO:
        // check number of cargo
        enableUpperBeamBreak(true);
        if (storedCargoColors[0] != CargoColor.NONE && storedCargoColors[1] != CargoColor.NONE) {
          logger.info("WAIT_CARGO -> STOP");
          currMagazineState = MagazineState.STOP;
          break;
        } else if (storedCargoColors[0] != CargoColor.NONE) {
          if (lowerMagazineTalon.getMotorOutputPercent() == 0.0) {
            lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIntakeSpeed);
          }
          // Checking if ball is in the top of the upper magazine
          if (upperMagazineTalon.getMotorOutputPercent() == 0.0) {
            upperClosedLoopRotate(MagazineConstants.kUpperMagazineIntakeSpeed);
          }
        }
        // Knowing when to read cargo color
        if (isLowerBeamBroken()) {
          currMagazineState = MagazineState.READ_CARGO;
          lowerOpenLoopRotate(0.0);
          logger.info("WAIT_CARGO -> READ_CARGO");
        } else if (lowerMagazineTalon.getMotorOutputPercent() == 0.0) {
          lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIntakeSpeed);
        }

        break;

      case READ_CARGO:
        // Read cargo color, switch states depending on amount of cargo
        CargoColor cargoColor = readCargoColor();
        if (cargoColor != CargoColor.NONE) {
          // ignoreColorSensor || storedCargoColors[0]
          if (cargoColor != allianceCargoColor && !ignoreColorSensor) {
            lowerClosedLoopRotate(MagazineConstants.kMagazineEjectSpeed);
            currMagazineState = MagazineState.EJECT_CARGO;
            ejectTimer.reset();
            ejectTimer.start();
            logger.info("READ_CARGO -> EJECT_CARGO");
            break;
          } else if (storedCargoColors[1] != CargoColor.NONE) {
            // if (storedCargoColors[1] != allianceCargoColor && (!ignoreColorSensor)) {
            // //storedCargoColors[1] != allianceColor
            //   lowerClosedLoopRotate(MagazineConstants.kMagazineEjectSpeed);
            //   currMagazineState = MagazineState.EJECT_CARGO;
            //   ejectTimer.reset();
            //   ejectTimer.start();
            //   logger.info("READ_CARGO -> EJECT_CARGO");
            //   break;
            // }
            currMagazineState = MagazineState.STOP;
            logger.info("READ_CARGO -> STOP");
            break;
          } else {
            lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIntakeSpeed);
            upperClosedLoopRotate(MagazineConstants.kUpperMagazineIntakeSpeed);
            currMagazineState = MagazineState.INDEX_CARGO;
            logger.info("READ_CARGO -> INDEX_CARGO");
            break;
          }
        }
        break;

      case INDEX_CARGO:
        if (!isLowerBeamBroken()) {
          enableUpperBeamBreak(true);
          currMagazineState = MagazineState.WAIT_CARGO;
          logger.info("INDEX_CARGO -> WAIT_CARGO");
        }
        break;

      case EJECT_CARGO:
        if (ejectTimer.hasElapsed(MagazineConstants.kEjectTimerDelay)) {
          if (storedCargoColors[1] == CargoColor.NONE) {
            storedCargoColors[0] = CargoColor.NONE;
          } else {
            storedCargoColors[1] = CargoColor.NONE;
          }
          currMagazineState = MagazineState.WAIT_CARGO;
          lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIntakeSpeed);
          logger.info("EJECT_CARGO -> WAIT_CARGO");
        }
        break;
      case PAUSE:
        if (shooterSubsystem.getCurrentState() == ShooterState.SHOOT
            && (turretSubsystem.getState() == TurretState.TRACKING
                || turretSubsystem.getState() == TurretState.FENDER_AIMED)) {
          logger.info("PAUSE -> SHOOT");
          enableUpperBeamBreak(false);
          upperClosedLoopRotate(MagazineConstants.kUpperMagazineFeedSpeed);
          currMagazineState = MagazineState.SHOOT;
        }
        break;
      case SHOOT:
        if (!isUpperBeamBroken()) shootUpperBeamStableCounts++;
        else shootUpperBeamStableCounts = 0;

        if (shootUpperBeamStableCounts > MagazineConstants.kShootUpperBeamStableCounts) {
          currMagazineState = MagazineState.CARGO_SHOT;
          shootTimer.reset();
          shootTimer.start();
          enableUpperBeamBreak(true);
          lowerClosedLoopRotate(MagazineConstants.kLowerMagazineIndexSpeed);
          upperClosedLoopRotate(MagazineConstants.kUpperMagazineIndexSpeed);
          shotOneCargo();
          logger.info("SHOOT -> CARGO_SHOT");
        }
        break;
      case CARGO_SHOT:
        if (shootTimer.hasElapsed(MagazineConstants.kShootDelay)
            && storedCargoColors[0] == CargoColor.NONE
            && !ignoreColorSensor) {
          currMagazineState = MagazineState.STOP;
          logger.info("CARGO_SHOT -> STOP");
          break;
        } else if (shootTimer.hasElapsed(MagazineConstants.kShootDelay) && isUpperBeamBroken()) {
          logger.info("CARGO_SHOT -> PAUSE");
          currMagazineState = MagazineState.PAUSE;
          enableUpperBeamBreak(true);
          if (turretSubsystem.getState() == TurretState.FENDER_AIMED) {
            shooterSubsystem.fenderShot();
            turretSubsystem.fenderShot();
          } else {
            shooterSubsystem.shoot();
          }
          lowerOpenLoopRotate(0.0);
          break;
        }
        break;

      case STOP:
        if (lowerMagazineTalon.getMotorOutputPercent() != 0.0) {
          stopMagazine();
        }
        break;
    }
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(lowerMagazineTalon);
    telemetryService.register(upperMagazineTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("red", () -> lastColor.red),
        new Measure("blue", () -> lastColor.blue),
        new Measure("green", () -> lastColor.green),
        new Measure("Proximity", () -> lastProximity));
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

  public enum MagazineState {
    MANUAL_INTAKE,
    WAIT_CARGO,
    READ_CARGO,
    INDEX_CARGO,
    EJECT_CARGO,
    SHOOT,
    CARGO_SHOT,
    PAUSE,
    STOP;
  }
}
