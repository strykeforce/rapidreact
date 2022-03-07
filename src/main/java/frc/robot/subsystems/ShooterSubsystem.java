package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.opencsv.CSVReader;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
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
  private final TalonFX kickerFalcon;
  private final TalonSRX hoodTalon;
  private final MagazineSubsystem magazineSubsystem;
  private final VisionSubsystem visionSubsystem;
  private boolean highFender;
  private ShooterState currentState = ShooterState.STOP;
  private double shooterSetPointTicks, kickerSetpointTicks, hoodSetPointTicks;
  private String[][] lookupTable;

  public ShooterSubsystem(MagazineSubsystem magazineSubsystem, VisionSubsystem visionSubsystem) {
    this.magazineSubsystem = magazineSubsystem;
    this.visionSubsystem = visionSubsystem;
    parseLookupTable();
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
    int index = 0;

    if (widthPixels < ShooterConstants.kLookupMinPixel) {
      logger.warn(
          "Pixel width {} is less than min pixel in table, using {}",
          widthPixels,
          ShooterConstants.kLookupMinPixel);
      index = 1;
    } else if (widthPixels > ShooterConstants.kLookupMaxPixel) {
      logger.warn(
          "Pixel width {} is more than max pixel in table, using {}",
          widthPixels,
          ShooterConstants.kLookupMaxPixel);
      index = lookupTable.length - 1;
    } else {
      index =
          (int)
              (Math.round(widthPixels / ShooterConstants.kLookupRes)
                  + 1
                  - ShooterConstants.kLookupMinPixel);
      logger.info("Selected Index: {}", index);
    }
    double[] shootSolution = new double[3];
    shootSolution[0] = Double.parseDouble(lookupTable[index][2]);
    shootSolution[1] = Double.parseDouble(lookupTable[index][3]);
    shootSolution[2] = Double.parseDouble(lookupTable[index][4]);
    logger.info(
        "Kicker Speed: {} Shooter Speed: {} Hood Pos: {}",
        shootSolution[0],
        shootSolution[1],
        shootSolution[2]);
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
    logger.info("Kicker at {} speed, Shooter at {} speed", kickerSpeed, shooterSpeed);
  }

  public void hoodClosedLoop(double hoodPos) {
    hoodTalon.set(ControlMode.MotionMagic, hoodPos);
    hoodSetPointTicks = hoodPos;
    logger.info("Hood is moving to {}", hoodPos);
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

  public void arm() {
    currentState = ShooterState.ARMING;
    shooterClosedLoop(
        ShooterConstants.kKickerArmTicksP100ms, ShooterConstants.kShooterArmTicksP100ms);
    logger.info("Arming starting");
  }

  public void shoot() {
    logger.info("SHOOT: {} -> ADJUSTING}", currentState);
    currentState = ShooterState.ADJUSTING;
    if (!magazineSubsystem.isNextCargoAlliance()) {
      shooterClosedLoop(
          ShooterConstants.kKickerOpTicksP100ms, ShooterConstants.kShooterOpTicksP100ms);
      hoodClosedLoop(ShooterConstants.kHoodOpTicks);
    } else {
      double[] shootSolution = getShootSolution();
      shooterClosedLoop(shootSolution[0], shootSolution[1]);
      hoodClosedLoop(shootSolution[2]);
    }
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

  public void stop() {
    logger.info("{} -> STOP", currentState);
    currentState = ShooterState.STOP;
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
        }
        break;
      case ARMING:
        if (isShooterAtSpeed()) {
          currentState = ShooterState.ARMED;
          logger.info("ARMING -> ARMED");
        }
        break;
      case ARMED:
        // Just indicates that it is ready to shoot for other classes
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
    return Set.of();
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
