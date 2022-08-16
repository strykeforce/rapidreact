package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.opencsv.CSVReader;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import java.io.FileReader;
import java.util.List;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ShooterSubsystem extends MeasurableSubsystem {

  private static final Logger logger = LoggerFactory.getLogger(ShooterSubsystem.class);
  private final TalonFX shooterFalcon;
  public final DriveSubsystem driveSubsystem;
  private final TalonFX kickerFalcon;
  private final TalonSRX hoodTalon;
  private final MagazineSubsystem magazineSubsystem;
  private final VisionSubsystem visionSubsystem;
  private boolean highFender;
  private boolean isBallOne;
  private ShooterState currentState = ShooterState.STOP;
  private double shooterSetPointTicks,
      kickerSetpointTicks,
      hoodSetPointTicks,
      logShooterTicks,
      logKickerTicks,
      logHoodTicks,
      oldWidthPixels,
      oldIndex;
  private String[][] lookupTable;
  private String[][] inchTable;
  public boolean isOutside = true;
  private double lastLookupDistance = 0.0;
  private boolean lastLookupBeyondTable = false;
  private Translation2d newHub = new Translation2d();
  private Translation2d delta = new Translation2d();
  private double changeInDistanceGoal = 0.0;

  public ShooterSubsystem(
      MagazineSubsystem magazineSubsystem,
      VisionSubsystem visionSubsystem,
      DriveSubsystem driveSubsystem) {
    this.magazineSubsystem = magazineSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    parseLookupTable();
    parseInchTable();
    shooterFalcon = new TalonFX(ShooterConstants.kShooterFalconID);
    shooterFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    shooterFalcon.configAllSettings(
        ShooterConstants.getShooterFalconConfig(), Constants.kTalonConfigTimeout);
    shooterFalcon.enableVoltageCompensation(true);
    shooterFalcon.setNeutralMode(NeutralMode.Coast);

    kickerFalcon = new TalonFX(ShooterConstants.kKickerFalconID);
    kickerFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    kickerFalcon.configAllSettings(
        ShooterConstants.getKickerFalconConfig(), Constants.kTalonConfigTimeout);
    kickerFalcon.enableVoltageCompensation(true);
    kickerFalcon.setNeutralMode(NeutralMode.Coast);

    hoodTalon = new TalonSRX(ShooterConstants.kHoodTalonID);
    hoodTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    hoodTalon.configAllSettings(
        ShooterConstants.getHoodTalonConfig(), Constants.kTalonConfigTimeout);
    hoodTalon.configSupplyCurrentLimit(
        ShooterConstants.getHoodCurrentLimit(), Constants.kTalonConfigTimeout);
    hoodTalon.enableVoltageCompensation(true);
    hoodTalon.setNeutralMode(NeutralMode.Brake);

    hoodTalon.configForwardSoftLimitEnable(true);
    hoodTalon.configForwardSoftLimitThreshold(ShooterConstants.kForwardSoftLimts);
    hoodTalon.configReverseSoftLimitEnable(true);
    hoodTalon.configReverseSoftLimitThreshold(ShooterConstants.kReverseSoftLimits);

    zeroHood();
  }

  public void zeroHood() {
    // double stringPotPosition = turret.getSensorCollection().getAnalogInRaw();
    if (!hoodTalon.getSensorCollection().isFwdLimitSwitchClosed()) {
      int absPos = hoodTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
      // inverted because absolute and relative encoders are out of phase
      int offset = absPos - ShooterConstants.kHoodZeroTicks;
      hoodTalon.setSelectedSensorPosition(offset);
      logger.info(
          "Hood zeroed; offset: {} zeroTicks: {} absPosition: {}",
          offset,
          ShooterConstants.kHoodZeroTicks,
          absPos);
    } else {
      hoodTalon.configPeakOutputForward(0, 0);
      hoodTalon.configPeakOutputReverse(0, 0);
      logger.error("Hood zero failed. Killing hood...");
    }
    hoodTalon.clearStickyFaults();
  }

  private void parseInchTable() {
    try {
      CSVReader csvReader = new CSVReader(new FileReader(ShooterConstants.kInchTablePath));

      List<String[]> list = csvReader.readAll();

      String[][] strArr = new String[list.size()][];
      inchTable = list.toArray(strArr);
    } catch (Exception exception) {
      logger.error("Could not read table at {}", ShooterConstants.kInchTablePath);
    }
  }

  private void parseLookupTable() {
    try {
      CSVReader csvReader = new CSVReader(new FileReader(ShooterConstants.kLookupTablePath));

      List<String[]> list = csvReader.readAll();

      String[][] strArr = new String[list.size()][];
      lookupTable = list.toArray(strArr);
    } catch (Exception exception) {
      logger.error("Could not read table at {}", ShooterConstants.kLookupTablePath);
    }
  }

  private double[] getShootSolution() {
    double widthPixels = visionSubsystem.getTargetPixelWidth();
    return getShootSolution(widthPixels);
  }

  private double[] getShootSolution(double widthPixels) {
    int index = 0;
    double[] shootSolution = new double[5];
    if (widthPixels < ShooterConstants.kLookupMinPixel) {
      logger.warn(
          "Pixel width {} is less than min pixel in table, using {}",
          widthPixels,
          ShooterConstants.kLookupMinPixel);
      index = lookupTable.length - 1;
      lastLookupBeyondTable = true;
    } else if (widthPixels > ShooterConstants.kLookupMaxPixel) {
      logger.warn(
          "Pixel width {} is more than max pixel in table, using {}",
          widthPixels,
          ShooterConstants.kLookupMaxPixel);
      index = 1;
      lastLookupBeyondTable = true;
    } else {
      // total rows - (Width - minrows)
      index =
          (int)
              (ShooterConstants.kNumRows
                  - (Math.round(widthPixels) - ShooterConstants.kLookupMinPixel));
      // index =
      //     (int)
      //         (Math.round(widthPixels / ShooterConstants.kLookupRes)
      //             + 1
      //             - ShooterConstants.kLookupMinPixel);
      oldIndex = index;
      oldWidthPixels = widthPixels;
      lastLookupBeyondTable = false;
    }

    shootSolution[0] = Double.parseDouble(lookupTable[index][2]);
    shootSolution[1] = Double.parseDouble(lookupTable[index][3]);
    shootSolution[2] = Double.parseDouble(lookupTable[index][4]);
    shootSolution[3] = Double.parseDouble(lookupTable[index][0]);
    shootSolution[4] = Double.parseDouble(lookupTable[index][5]);
    return shootSolution;
  }

  public void shooterOpenLoop(double speed) {
    logger.info("Shooter on {}", speed);
    shooterFalcon.set(ControlMode.PercentOutput, speed);
    kickerFalcon.set(ControlMode.PercentOutput, speed);
    if (speed == 0.0) currentState = ShooterState.STOP;
    else currentState = ShooterState.MANUAL_SHOOT;
  }

  public void hoodOpenLoop(double speed) {
    logger.info("hood on {}", speed);
    hoodTalon.set(ControlMode.PercentOutput, speed);
  }

  public void manualClosedLoop(double kickerSpeed, double shooterSpeed) {
    currentState = ShooterState.MANUAL_SHOOT;
    shooterClosedLoop(kickerSpeed, shooterSpeed);
  }

  public void shooterClosedLoop(double kickerSpeed, double shooterSpeed) {
    shooterFalcon.set(ControlMode.Velocity, shooterSpeed);
    kickerFalcon.set(ControlMode.Velocity, kickerSpeed);
    shooterSetPointTicks = shooterSpeed;
    kickerSetpointTicks = kickerSpeed;
    logShooterTicks = shooterSpeed;
    logKickerTicks = kickerSpeed;
  }

  public void logShotSol() {
    logger.info(
        "Kicker setpoint {}, Shooter setpoint {}, Hood setpoint {}",
        logKickerTicks,
        logShooterTicks,
        logHoodTicks);
    logger.info(
        "Kicker at {} speed, Shooter at {} speed, Hood at {} pos",
        kickerFalcon.getSelectedSensorVelocity(),
        shooterFalcon.getSelectedSensorVelocity(),
        hoodTalon.getSelectedSensorPosition());
    logger.info("Selected Index: {}, widthPixels: {}", oldIndex, oldWidthPixels);
  }

  public void hoodClosedLoop(double hoodPos) {
    hoodTalon.set(ControlMode.MotionMagic, hoodPos);
    hoodSetPointTicks = hoodPos;
    logHoodTicks = hoodSetPointTicks;
  }

  public boolean isShooterAtSpeed() {
    return (Math.abs(shooterSetPointTicks - shooterFalcon.getSelectedSensorVelocity())
            < ShooterConstants.kCloseEnoughTicksP100ms
        && Math.abs(kickerSetpointTicks - kickerFalcon.getSelectedSensorVelocity())
            < ShooterConstants.kCloseEnoughTicksP100ms);
  }

  public boolean isHoodAtPos() {
    return (Math.abs(hoodSetPointTicks - hoodTalon.getSelectedSensorPosition())
        < ShooterConstants.kCloseEnoughTicks);
  }

  public ShooterState getCurrentState() {
    return currentState;
  }

  public List<BaseTalon> getShooterTalons() {
    return List.of(kickerFalcon, shooterFalcon);
  }

  public List<BaseTalon> getHoodTalons() {
    return List.of(hoodTalon);
  }

  public void arm() {
    currentState = ShooterState.ARMING;
    shooterClosedLoop(
        ShooterConstants.kKickerArmTicksP100ms, ShooterConstants.kShooterArmTicksP100ms);
    logger.info("Arming starting");
  }

  public int inchesToPixelsTable(Translation2d newHub) {
    double inches = Math.round(driveSubsystem.getDistToTranslation2d(newHub) * 2) / 2;
    logger.info("inches:  {}", inches);
    if (inches < ShooterConstants.kLookupMinInch) {
      return (int) Double.parseDouble(inchTable[1][1]);
    }
    if (inches > ShooterConstants.kLookupMaxInch) {
      return (int) Double.parseDouble(inchTable[(int) ShooterConstants.kLookupInchMaxIndex][1]);
    }
    double index = 0;
    index =
        (inches - (ShooterConstants.kLookupMinInch - ShooterConstants.kLookupMinInchChange))
            / ShooterConstants.kLookupMinInchChange;
    logger.info(
        "Inches: {} | Pixel: {}",
        driveSubsystem.getDistToTranslation2d(newHub),
        Double.parseDouble(inchTable[(int) index][1]));
    return (int) Double.parseDouble(inchTable[(int) index][1]);
  }

  public Translation2d getFutureGoalPos(double time) {
    if (currentState != ShooterState.SHOOT) {
      logger.info("SHOOT: {} -> ADJUSTING", currentState);
      currentState = ShooterState.ADJUSTING;
    }
    if (!magazineSubsystem.isNextCargoAlliance()) {
      shooterClosedLoop(
          ShooterConstants.kKickerOpTicksP100ms, ShooterConstants.kShooterOpTicksP100ms);
      hoodClosedLoop(ShooterConstants.kHoodOpTicks);
    } else {
      double[] shootSolution =
          getShootSolution(inchesToPixelsTable(TurretConstants.kHubPositionMeters));
      double dx =
          -driveSubsystem.getFieldRelSpeed().vxMetersPerSecond
              * ((shootSolution[4] - time + ShooterConstants.kLookupTOFOffset)
                  * ShooterConstants.kLookupToFMultiplier);
      double dy =
          -driveSubsystem.getFieldRelSpeed().vyMetersPerSecond
              * ((shootSolution[4] - time + ShooterConstants.kLookupTOFOffset)
                  * ShooterConstants.kLookupToFMultiplier);
      delta = new Translation2d(dx, dy);
      newHub = TurretConstants.kHubPositionMeters;
      newHub = newHub.plus(delta);
      double lastDistance = shootSolution[3];
      shootSolution = getShootSolution(inchesToPixelsTable(newHub));
      changeInDistanceGoal = shootSolution[3] - lastDistance;
        shooterClosedLoop(shootSolution[0], shootSolution[1]);
        hoodClosedLoop(shootSolution[2]);
      return newHub;
    }
    return TurretConstants.kHubPositionMeters;
  }

  public double getDeltaGoalDistance() {
    return changeInDistanceGoal;
  }

  public void shoot() {
    if (currentState != ShooterState.SHOOT) {
      logger.info("SHOOT: {} -> ADJUSTING", currentState);
      currentState = ShooterState.ADJUSTING;
    }
    if (!magazineSubsystem.isNextCargoAlliance()) {
      shooterClosedLoop(
          ShooterConstants.kKickerOpTicksP100ms, ShooterConstants.kShooterOpTicksP100ms);
      hoodClosedLoop(ShooterConstants.kHoodOpTicks);
    } else {
      if (visionSubsystem.isRangingValid()) {
        double[] shootSolution = getShootSolution();
        lastLookupDistance = shootSolution[3];
        shooterClosedLoop(shootSolution[0], shootSolution[1]);
        hoodClosedLoop(shootSolution[2]);
      }
    }
  }

  public double getDistInches() {
    if (visionSubsystem.isRangingValid()) {
      double[] shootSolution = getShootSolution();
      return shootSolution[3];
    }
    return 2767.0;
  }

  public void strykeShot() {
    logger.info("Stryke Shot: {} -> ADJUSTING", currentState);
    currentState = ShooterState.ADJUSTING;
    shooterClosedLoop(
        isOutside
            ? ShooterConstants.kOutsideKickerTicksP100MS
            : ShooterConstants.kInsideKickerTicksP100MS,
        isOutside
            ? ShooterConstants.kOutsideShooterTicksP100MS
            : ShooterConstants.kInsideShooterTicksP100MS);
    hoodClosedLoop(
        isOutside ? ShooterConstants.kOutsideHoodTickPos : ShooterConstants.kInsideHoodTickPos);
  }

  public void setOutside(boolean pos) {
    isOutside = pos;
    logger.info("Manually Set Climb Pos: {}", pos ? "LEFT" : "RIGHT");
  }

  public void checkOutside() {
    Pose2d robotPos = driveSubsystem.getPoseMeters();
    if (robotPos.getY() > ShooterConstants.kMiddleClimbY) {
      isOutside = true;
      logger.info("Detect OUTSIDE climb: y= {}", robotPos.getY());
    } else {
      isOutside = false;
      logger.info("Detect INSIDE climb: y= {}", robotPos.getY());
    }
  }

  public String getIsOutside() {
    return isOutside ? "OUTSIDE" : "INSIDE";
  }

  public double getLastLookupDistance() {
    return lastLookupDistance;
  }

  public boolean isLastLookupBeyondTable() {
    return lastLookupBeyondTable;
  }

  public void manualShoot(double widthPixels) {
    logger.info("No vision: SHOOT: {} -> ADJUSTING", currentState);
    currentState = ShooterState.ADJUSTING;
    double[] shootSolution = getShootSolution(widthPixels);
    shooterClosedLoop(shootSolution[0], shootSolution[1]);
    hoodClosedLoop(shootSolution[2]);
  }

  public void fenderShot() {
    fenderShot(highFender);
  }

  public void fenderShot(boolean highFender) {
    this.highFender = highFender;
    logger.info("FENDER_SHOT: {} -> ADJUSTING", currentState);
    currentState = ShooterState.ADJUSTING;
    if (!magazineSubsystem.isNextCargoAlliance()) {
      shooterClosedLoop(
          ShooterConstants.kKickerOpTicksP100ms, ShooterConstants.kShooterOpTicksP100ms);
      hoodClosedLoop(ShooterConstants.kHoodOpTicks);
      logger.info("Fender Shot: Opponent Setpoints");
    } else if (highFender) {
      shooterClosedLoop(
          ShooterConstants.kKickerFenderHighTicksP100ms,
          ShooterConstants.kShooterFenderHighTicksP100ms);
      hoodClosedLoop(ShooterConstants.kHoodFenderHighTicks);
      logger.info("High Fender Shot");
    } else {
      shooterClosedLoop(
          ShooterConstants.kKickerFenderLowTicksP100ms,
          ShooterConstants.kShooterFenderLowTicksP100ms);
      hoodClosedLoop(ShooterConstants.kHoodFenderLowTicks);
      logger.info("Low Fender Shot");
    }
  }

  public void geyserShot(boolean isOppCargoShot, boolean isBallOne) {
    this.isBallOne = isBallOne;
    logger.info("GEYSER_SHOT: {} -> ADJUSTING", currentState);
    currentState = ShooterState.ADJUSTING;
    if (!isOppCargoShot) {
      shooterClosedLoop(
          ShooterConstants.kKickerGeyserTicksP100ms, ShooterConstants.kShooterGeyserTicksP100ms);
      if (isBallOne) {
        hoodClosedLoop(ShooterConstants.kHoodGeyserBallTwoTicks);
      } else {
        hoodClosedLoop(ShooterConstants.kHoodGeyserBallTwoTicks);
      }
    } else {
      double[] shootSol =
          getShootSolution(inchesToPixelsTable(ShooterConstants.kOpponentCargoShotSol));
      shooterClosedLoop(shootSol[0], shootSol[1]);
      hoodClosedLoop(shootSol[2]);
    }
  }

  public void geyserShot() {
    geyserShot(false, !isBallOne);
  }

  public void stop() {
    logger.info("{} -> STOP", currentState);
    currentState = ShooterState.STOP;
  }

  public void setManualState() {
    currentState = ShooterState.MANUAL_SHOOT;
    logger.info("SetManualState ");
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case MANUAL_SHOOT:
        break;
      case STOP:
        if (kickerFalcon.getMotorOutputPercent() != 0.0
            || shooterFalcon.getMotorOutputPercent() != 0.0) {
          shooterOpenLoop(0.0);
          hoodClosedLoop(0);
        }
        break;
      case ARMING:
        if (isShooterAtSpeed()) {
          currentState = ShooterState.ARMED;
          logger.info("ARMING -> ARMED");
        }
        break;
      case ARMED:
        if (visionSubsystem.isRangingValid()) {
          double[] shootSolution = getShootSolution();
          shooterClosedLoop(shootSolution[0], shootSolution[1]);
        }
        break;
      case ADJUSTING:
        if (isHoodAtPos() && isShooterAtSpeed()) {
          currentState = ShooterState.SHOOT;
          logger.info("ADJUSTING -> SHOOT");
        }
        break;
      case SHOOT:
        // Just a state that lets other classes it needs to shoot
        break;
    }
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(shooterFalcon);
    telemetryService.register(kickerFalcon);
    telemetryService.register(hoodTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Shooter State", () -> currentState.ordinal()),
        new Measure("Future Goal X", () -> newHub.getX()),
        new Measure("Future Goal Y", () -> newHub.getY()),
        new Measure("Delta Goal X", () -> delta.getX()),
        new Measure("Delta Goal Y", () -> delta.getY()),
        new Measure("Change in Goal", () -> changeInDistanceGoal));
  }

  public enum ShooterState {
    STOP,
    ARMING,
    ARMED,
    ADJUSTING,
    SHOOT,
    MANUAL_SHOOT;
  }
}
