package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
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
  private static final Logger logger = LoggerFactory.getLogger(MeasurableSubsystem.class);
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
  private Timer timer = new Timer();
  private boolean ignoreColorSensor = false;

  public MagazineSubsystem(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    // colorSensor  = new ColorSensorV3(Port.kMXP);

    // lowerMagazineTalon = new TalonSRX(MagazineConstants.kLowerMagazineTalonID);
    // lowerMagazineTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    // lowerMagazineTalon.configAllSettings(
    //     MagazineConstants.getMagazineTalonConfig(), Constants.kTalonConfigTimeout);
    // lowerMagazineTalon.enableCurrentLimit(true);
    // lowerMagazineTalon.enableVoltageCompensation(true);
    // lowerMagazineTalon.setNeutralMode(NeutralMode.Coast);

    // upperMagazineTalon = new TalonSRX(MagazineConstants.kUpperMagazineTalonID);
    // upperMagazineTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    // upperMagazineTalon.configAllSettings(
    //     MagazineConstants.getMagazineTalonConfig(), Constants.kTalonConfigTimeout);
    // upperMagazineTalon.enableCurrentLimit(true);
    // upperMagazineTalon.enableVoltageCompensation(true);
    // upperMagazineTalon.setNeutralMode(NeutralMode.Coast);

    colorMatch.addColorMatch(MagazineConstants.kBlueCargo);
    colorMatch.addColorMatch(MagazineConstants.kRedCargo);
    colorMatch.addColorMatch(MagazineConstants.kNoCargo);
  }

  public void setShooterSubsystem(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
  }

  public void lowerOpenLoopRotate(double percentOutput) {
    // lowerMagazineTalon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Lower magazine motor turned on {}", percentOutput);
  }

  public void upperOpenLoopRotate(double percentOutput) {
    // upperMagazineTalon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Upper magazine motor turned on {}", percentOutput);
  }

  public void stopMagazine() {
    // lowerMagazineTalon.set(ControlMode.PercentOutput, 0.0);
    // upperMagazineTalon.set(ControlMode.PercentOutput, 0.0);
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

  public boolean isNextCargoAlliance() {
    return ignoreColorSensor || storedCargoColors[0] == allianceCargoColor;
  }

  public CargoColor readCargoColor() {
    Color readColor = getColor();
    ColorMatchResult matchResult = colorMatch.matchClosestColor(readColor);

    CargoColor currentCargoColor = CargoColor.NONE;
    if (matchResult.color == MagazineConstants.kBlueCargo) {
      currentCargoColor = CargoColor.BLUE;
    } else if (matchResult.color == MagazineConstants.kRedCargo) {
      currentCargoColor = CargoColor.RED;
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

  public void indexCargo() {
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
    lowerOpenLoopRotate(upperSpeed);
  }

  private void autoStopUpperMagazine(double speed) {
    if (isUpperBeamBroken() && upperMagazineTalon.getMotorOutputPercent() != 0.0) {
      upperOpenLoopRotate(0.0);
      logger.info("Stopping upper magazine, upper beam broken");
    } else if (!isUpperBeamBroken() && upperMagazineTalon.getMotorOutputPercent() == 0.0) {
      // upperOpenLoopRotate(speed);
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
    logger.info("{} -> PAUSE}", currMagazineState);
    currMagazineState = MagazineState.PAUSE;
  }

  @Override
  public void periodic() {
    switch (currMagazineState) {
      case MANUAL_INTAKE:
        break;

      case WAIT_CARGO:
        // check number of cargo
        if (storedCargoColors[0] != CargoColor.NONE && storedCargoColors[1] != CargoColor.NONE) {
          logger.info("Magazine already full, switching to stop state");
          currMagazineState = MagazineState.STOP;
          break;
        } else if (storedCargoColors[0] != CargoColor.NONE) {
          if (lowerMagazineTalon.getMotorOutputPercent() == 0.0) {
            // lowerOpenLoopRotate(MagazineConstants.kMagazineIntakeSpeed);
          }
          // Checking if ball is in the top of the upper magazine
          autoStopUpperMagazine(MagazineConstants.kMagazineIntakeSpeed);
        }
        // Knowing when to read cargo color
        if (isLowerBeamBroken()) {
          currMagazineState = MagazineState.READ_CARGO;
          // lowerOpenLoopRotate(0.0);
          logger.info("Switching state to read cargo color");
        } else if (lowerMagazineTalon.getMotorOutputPercent() == 0.0) {
          // lowerOpenLoopRotate(MagazineConstants.kMagazineIntakeSpeed);
        }

        break;

      case READ_CARGO:
        // Read cargo color, switch states depending on amount of cargo
        CargoColor cargoColor = readCargoColor();
        if (cargoColor != CargoColor.NONE) {
          if (storedCargoColors[1] != CargoColor.NONE) {
            currMagazineState = MagazineState.STOP;
            logger.info("Now have two cargo, switching to stop state");
          } else {
            // lowerOpenLoopRotate(MagazineConstants.kMagazineIntakeSpeed);
            // upperOpenLoopRotate(MagazineConstants.kMagazineIntakeSpeed);
            currMagazineState = MagazineState.INDEX_CARGO;
            logger.info("Has one cargo, turning on magazine, switching to index state");
          }
        }
        autoStopUpperMagazine(MagazineConstants.kMagazineIntakeSpeed);
        break;

      case INDEX_CARGO:
        if (!isLowerBeamBroken()) {
          currMagazineState = MagazineState.WAIT_CARGO;
          logger.info("Switching to wait state");
        }
        autoStopUpperMagazine(MagazineConstants.kMagazineIntakeSpeed);
        break;

      case PAUSE:
        if (shooterSubsystem.getCurrentState() == ShooterState.SHOOT
            && turretSubsystem.getState() == TurretState.TRACKING) {
          logger.info("PAUSE -> SHOOT");
          upperOpenLoopRotate(MagazineConstants.kMagazineFeedSpeed);
          currMagazineState = MagazineState.SHOOT;
        }
        break;
      case SHOOT:
        if (!isUpperBeamBroken()) {
          currMagazineState = MagazineState.CARGO_SHOT;
          timer.reset();
          timer.start();
          lowerOpenLoopRotate(MagazineConstants.kMagazineIndexSpeed);
          upperOpenLoopRotate(MagazineConstants.kMagazineIndexSpeed);
          shotOneCargo();
          logger.info("SHOOT -> CARGO_SHOT");
        }
        break;
      case CARGO_SHOT:
        if (timer.hasElapsed(MagazineConstants.kShootDelay) && isUpperBeamBroken()) {
          logger.info("CARGO_SHOT -> PAUSE");
          currMagazineState = MagazineState.PAUSE;
          if (turretSubsystem.getState() == TurretState.FENDER_AIMED) {
            shooterSubsystem.fenderShot();
            turretSubsystem.fenderShot();
          } else {
            shooterSubsystem.shoot();
          }
          upperOpenLoopRotate(0.0);
          lowerOpenLoopRotate(0.0);
          break;
        } else if (timer.hasElapsed(MagazineConstants.kShootDelay)
            && storedCargoColors[0] == CargoColor.NONE) {
          currMagazineState = MagazineState.STOP;
          logger.info("CARGO_SHOT -> STOP");
          break;
        }
        autoStopUpperMagazine(MagazineConstants.kMagazineIndexSpeed);
        break;

      case STOP:
        stopMagazine();
        break;
    }
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    // telemetryService.register(lowerMagazineTalon);
    // telemetryService.register(upperMagazineTalon);
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
    SHOOT,
    CARGO_SHOT,
    PAUSE,
    STOP;
  }
}
