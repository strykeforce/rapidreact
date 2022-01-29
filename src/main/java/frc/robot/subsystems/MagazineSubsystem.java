package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.MagazineConstants;
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
  private ColorMatch colorMatch = new ColorMatch();
  private MagazineState currMagazineState = MagazineState.STOP;

  public MagazineSubsystem() {
    // colorSensor = new ColorSensorV3(Port.kMXP);

    lowerMagazineTalon = new TalonSRX(MagazineConstants.LowerMagazineTalonID);
    lowerMagazineTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    lowerMagazineTalon.configAllSettings(
        MagazineConstants.getMagazineTalonConfig(), Constants.kTalonConfigTimeout);
    lowerMagazineTalon.enableCurrentLimit(true);
    lowerMagazineTalon.enableVoltageCompensation(true);
    lowerMagazineTalon.setNeutralMode(NeutralMode.Coast);

    upperMagazineTalon = new TalonSRX(MagazineConstants.UpperMagazineTalonID);
    upperMagazineTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    upperMagazineTalon.configAllSettings(
        MagazineConstants.getMagazineTalonConfig(), Constants.kTalonConfigTimeout);
    upperMagazineTalon.enableCurrentLimit(true);
    upperMagazineTalon.enableVoltageCompensation(true);
    upperMagazineTalon.setNeutralMode(NeutralMode.Coast);

    colorMatch.addColorMatch(MagazineConstants.kBlueCargo);
    colorMatch.addColorMatch(MagazineConstants.kRedCargo);
    colorMatch.addColorMatch(MagazineConstants.kNoCargo);
  }

  public void lowerOpenLoopRotate(double percentOutput) {
    // lowerMagazineTalon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Lower magazine motor turned on {}", percentOutput);
  }

  public void upperOpenLoopRotate(double percentOutput) {
    // upperMagazineTalon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("Upper magazine motor turned on {}", percentOutput);
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
    // lastColor = colorSensor.getColor();
    lastColor = new Color(0, 0, 0);
    return lastColor;
  }

  public int getProximity() {
    lastProximity = colorSensor.getProximity();
    return lastProximity;
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

  public void indexCargo()
  {
    logger.info("Start indexing cargo");
    currMagazineState = MagazineState.WAIT_CARGO;
  }

  @Override
  public void periodic() 
  {
    switch (currMagazineState) {
      case WAIT_CARGO: 
        // check number of cargo
        if(storedCargoColors[0] != CargoColor.NONE && storedCargoColors[1] != CargoColor.NONE)
        {
          logger.info("Magazine already full");
          currMagazineState = MagazineState.STOP;
        }
        else if(storedCargoColors[0] != CargoColor.NONE)
        {
          lowerOpenLoopRotate(MagazineConstants.kMagazineIntakeSpeed);
          if(isUpperBeamBroken() && upperMagazineTalon.getMotorOutputPercent() != 0.0)
          {
            upperOpenLoopRotate(0.0);
            logger.info("Stopping upper magazine, upper beam broken");
          }
          else
          {
            upperOpenLoopRotate(MagazineConstants.kMagazineIntakeSpeed);
          }
        }
        break;

      case READ_CARGO:

      break;

      case STOP:

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
    RED,
    BLUE,
    NONE;
  }

  public enum MagazineState
  {
    WAIT_CARGO,
    READ_CARGO,
    STOP;
  }
}

