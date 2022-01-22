package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;
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
  private TalonSRX magazineTalon;
  private CargoColor[] storedCargoColors = new CargoColor[] {CargoColor.NONE, CargoColor.NONE};
  private ColorMatch colorMatch = new ColorMatch();

  public MagazineSubsystem() {
    colorSensor = new ColorSensorV3(Port.kMXP);
    magazineTalon = new TalonSRX(MagazineConstants.MagazineTalonID);
    magazineTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    magazineTalon.configAllSettings(
        MagazineConstants.getMagazineTalonConfig(), Constants.kTalonConfigTimeout);
    magazineTalon.enableCurrentLimit(true);
    magazineTalon.enableVoltageCompensation(true);
    magazineTalon.setNeutralMode(NeutralMode.Coast);

    colorMatch.addColorMatch(MagazineConstants.kBlueCargo);
    colorMatch.addColorMatch(MagazineConstants.kRedCargo);
  }

  public void openLoopRotate(double percentOutput) {
    magazineTalon.set(ControlMode.PercentOutput, percentOutput);
  }

  public Color getColor() {
    lastColor = colorSensor.getColor();
    return lastColor;
  }

  public void readCargoColor() {
    Color readColor = getColor();
    ColorMatchResult matchResult = colorMatch.matchClosestColor(readColor);

    CargoColor currentCargoColor = CargoColor.NONE;
    if (matchResult.color == MagazineConstants.kBlueCargo) {
      currentCargoColor = CargoColor.BLUE;
    } else if (matchResult.color == MagazineConstants.kRedCargo) {
      currentCargoColor = CargoColor.RED;
    }

    if (storedCargoColors[0] != CargoColor.NONE) {
      storedCargoColors[0] = currentCargoColor;
      logger.info("Added first cargo {}", currentCargoColor);
    } else if (storedCargoColors[1] != CargoColor.NONE) {
      storedCargoColors[1] = currentCargoColor;
      logger.info("Added second cargo {}", currentCargoColor);
    } else {
      logger.error("Picked up third cargo {}, not recording", currentCargoColor);
    }
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

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(magazineTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("red", () -> lastColor.red),
        new Measure("blue", () -> lastColor.blue),
        new Measure("green", () -> lastColor.green));
  }

  public enum CargoColor {
    RED,
    BLUE,
    NONE;
  }
}
