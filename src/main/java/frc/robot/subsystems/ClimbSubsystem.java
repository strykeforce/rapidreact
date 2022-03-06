package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ClimbSubsystem extends MeasurableSubsystem {
  private static final Logger logger = LoggerFactory.getLogger(ClimbSubsystem.class);

  private final TalonFX pivotArmFalcon;
  private final TalonFX fixedArmFalcon;
  private final TalonSRX shoulderTalon;
  private final Servo pivotRatchet;
  private final Servo fixedRatchet;
  private final DigitalInput leftArmHome;
  private final DigitalInput rightArmHome;

  private FixedArmState currFixedArmState = FixedArmState.IDLE;
  private FixedArmState desiredFixedArmState = FixedArmState.IDLE;
  private PivotArmState currPivotArmState = PivotArmState.IDLE;
  private PivotArmState desiredPivotArmState = PivotArmState.IDLE;
  private ShoulderState shoulderState = ShoulderState.IDLE;
  private int fixedArmStableCounts = 0;
  private int pivotArmStableCounts = 0;
  private int shoulderStableCounts = 0;
  private int fixedArmZeroStableCounts = 0;
  private int pivotArmZeroStableCounts;
  private boolean continueToHigh = false;
  private boolean continueToTraverse = false;
  private boolean isClimbDone = false;
  private boolean isClimbInitiated = false;
  private Timer disengageFixedArmRatchetTimer = new Timer();
  private Timer disengagePivotArmRatchetTimer = new Timer();
  private int leftHomeStableCounts = 0;
  private int rightHomeStableCounts = 0;
  private boolean motorOnDisengageFixedRatchet = false;
  private boolean motorOnDisengagePivotRatchet = false;

  private double pivotArmSetPointTicks;
  private double fixedArmSetPointTicks;
  private double shoulderSetPointTicks;
  private double pivotArmDisengageRatchetStartTicks = 0;
  private double fixedArmDisengageRatchetStartTicks = 0;
  private boolean isPivotRatchetOn = false;
  private boolean isFixedRatchetOn = false;

  public ClimbSubsystem() {
    pivotArmFalcon = new TalonFX(ClimbConstants.kPivotArmFalconID);
    fixedArmFalcon = new TalonFX(ClimbConstants.kFixedArmFalconID);
    shoulderTalon = new TalonSRX(ClimbConstants.kClimbShoulderId);
    pivotRatchet = new Servo(ClimbConstants.kPivotArmRatchetId);
    fixedRatchet = new Servo(ClimbConstants.kFixedArmRatchetId);
    leftArmHome = new DigitalInput(ClimbConstants.kLeftFixedHomeId);
    rightArmHome = new DigitalInput(ClimbConstants.kRightFixedHomeId);
    enablePivotRatchet(false);
    enableFixedRatchet(false);

    configTalons();
  }

  private void configTalons() {
    pivotArmFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    pivotArmFalcon.configAllSettings(
        ClimbConstants.getPivotArmFalconConfig(), Constants.kTalonConfigTimeout);
    pivotArmFalcon.enableVoltageCompensation(true);
    pivotArmFalcon.setNeutralMode(NeutralMode.Brake);

    fixedArmFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    fixedArmFalcon.configAllSettings(
        ClimbConstants.getFixedArmFalconConfig(), Constants.kTalonConfigTimeout);
    fixedArmFalcon.enableVoltageCompensation(true);
    fixedArmFalcon.setNeutralMode(NeutralMode.Brake);

    shoulderTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
    shoulderTalon.configAllSettings(
        ClimbConstants.getShoulderTalonConfig(), Constants.kTalonConfigTimeout);
    shoulderTalon.configSupplyCurrentLimit(
        ClimbConstants.getShoulderCurrentLimit(), Constants.kTalonConfigTimeout);
    shoulderTalon.enableVoltageCompensation(true);
    shoulderTalon.setNeutralMode(NeutralMode.Brake);
  }

  public void openLoopShoulder(double speed) {
    logger.info("Shoulder is rotating: {} ticks per 100 ms", speed);
    shoulderTalon.set(ControlMode.PercentOutput, speed);
  }

  public void openLoopPivotArm(double speed) {
    logger.info("Pivot arms: {} ticks per 100 ms", speed);
    pivotArmFalcon.set(ControlMode.PercentOutput, speed);
  }

  public void openLoopFixedArm(double speed) {
    logger.info("Fixed arms: {} ticks per 100 ms", speed);
    fixedArmFalcon.set(ControlMode.PercentOutput, speed);
  }

  public void offsetShoulder(double offset) {
    rotateShoulder(shoulderSetPointTicks + offset);
  }

  public void rotateShoulder(double setPointsTicks) {
    logger.info("Shoulder is rotating to {} ticks", setPointsTicks);
    shoulderSetPointTicks = setPointsTicks;
    shoulderTalon.set(ControlMode.MotionMagic, setPointsTicks);
  }

  public void actuatePivotArm(double setPointTicks) {
    logger.info("Pivot arm is going to {} ticks", setPointTicks);
    pivotArmSetPointTicks = setPointTicks;
    pivotArmFalcon.set(ControlMode.MotionMagic, setPointTicks);
  }

  public void actuateFixedArm(double setPointTicks) {
    logger.info("Fixed arm is going to {} ticks", setPointTicks);
    fixedArmSetPointTicks = setPointTicks;
    fixedArmFalcon.set(ControlMode.MotionMagic, setPointTicks);
  }

  public void toggleFixedArmRatchet() {
    if (isFixedRatchetOn) fixedRatchet.set(ClimbConstants.kFixedRatchetOff);
    else fixedRatchet.set(ClimbConstants.kFixedRatchetOn);
    isFixedRatchetOn = !isFixedRatchetOn;
    logger.info("Setting Fixed Ratchet to: {}", isFixedRatchetOn);
  }

  public void togglePivotArmRatchet() {
    if (isPivotRatchetOn) pivotRatchet.set(ClimbConstants.kPivotRatchetOff);
    else pivotRatchet.set(ClimbConstants.kPivotRatchetOn);
    isPivotRatchetOn = !isPivotRatchetOn;
    logger.info("Setting Rotating Ratchet to {}", isPivotRatchetOn);
  }

  public void enableFixedRatchet(boolean enable) {
    if (enable) fixedRatchet.set(ClimbConstants.kFixedRatchetOn);
    else fixedRatchet.set(ClimbConstants.kFixedRatchetOff);
    isFixedRatchetOn = enable;
    logger.info("Setting Fixed Ratchet to: {}", isFixedRatchetOn);
  }

  public void enablePivotRatchet(boolean enable) {
    if (enable) pivotRatchet.set(ClimbConstants.kPivotRatchetOn);
    else pivotRatchet.set(ClimbConstants.kPivotRatchetOff);
    isPivotRatchetOn = enable;
    logger.info("Setting Pivot Ratchet to: {}", isPivotRatchetOn);
  }

  public void disengageFixedRatchet() {
    disengageFixedArmRatchetTimer.reset();
    disengageFixedArmRatchetTimer.start();
    fixedArmDisengageRatchetStartTicks = fixedArmFalcon.getSelectedSensorPosition();
    enableFixedRatchet(false);
    motorOnDisengageFixedRatchet = false;
    // openLoopFixedArm(ClimbConstants.kDisengageRatchetSpeed);
  }

  public void disengagePivotRatchet() {
    disengagePivotArmRatchetTimer.reset();
    disengagePivotArmRatchetTimer.start();
    pivotArmDisengageRatchetStartTicks = pivotArmFalcon.getSelectedSensorPosition();
    enablePivotRatchet(false);
    motorOnDisengagePivotRatchet = false;
    // openLoopPivotArm(ClimbConstants.kDisengageRatchetSpeed);
  }

  public boolean isFixedArmOpenLoopExtendFinished(double setpointTicks) {
    double currFixedArmPosTicks = fixedArmFalcon.getSelectedSensorPosition();
    return currFixedArmPosTicks < setpointTicks;
  }

  public boolean isFixedArmOpenLoopRetractFinished(double setpointTicks) {
    double currFixedArmPosTicks = fixedArmFalcon.getSelectedSensorPosition();
    return currFixedArmPosTicks > setpointTicks;
  }

  public boolean isPivotArmOpenLoopExtendFinished(double setpointTicks) {
    double currPivotArmPosTicks = pivotArmFalcon.getSelectedSensorPosition();
    return currPivotArmPosTicks < setpointTicks;
  }

  public boolean isPivotArmOpenLoopRetractFinished(double setpointTicks) {
    double currPivotArmPosTicks = pivotArmFalcon.getSelectedSensorPosition();
    return currPivotArmPosTicks > setpointTicks;
  }

  public boolean isFixedArmFinished() {
    double currFixedArmPosTicks = fixedArmFalcon.getSelectedSensorPosition();
    if (Math.abs(currFixedArmPosTicks - fixedArmSetPointTicks)
        > ClimbConstants.kFixedArmCloseEnough) {
      fixedArmStableCounts = 0;
    } else {
      fixedArmStableCounts++;
    }
    return fixedArmStableCounts >= ClimbConstants.kFixedArmStableCounts;
  }

  public boolean isPivotArmFinished() {
    double currPivotArmPosTicks = pivotArmFalcon.getSelectedSensorPosition();
    if (Math.abs(currPivotArmPosTicks - pivotArmSetPointTicks)
        > ClimbConstants.kPivotArmCloseEnough) {
      pivotArmStableCounts = 0;
    } else {
      pivotArmStableCounts++;
    }
    return pivotArmStableCounts >= ClimbConstants.kPivotArmStableCounts;
  }

  public boolean isShoulderFinished() {
    double currShoulderPosTicks = shoulderTalon.getSelectedSensorPosition();
    if (Math.abs(currShoulderPosTicks - shoulderSetPointTicks)
        > ClimbConstants.kShoulderCloseEnough) {
      shoulderStableCounts = 0;
    } else {
      shoulderStableCounts++;
    }
    return shoulderStableCounts >= ClimbConstants.kShoulderStableCounts;
  }

  public void testZeroClimb() {
    pivotArmFalcon.setSelectedSensorPosition(0.0);
    fixedArmFalcon.setSelectedSensorPosition(0.0);
    shoulderTalon.setSelectedSensorPosition(0.0);
    logger.info("Zeroing all Climb axes");
  }

  public void zeroClimb() {
    currPivotArmState = PivotArmState.ZEROING;
    currFixedArmState = FixedArmState.ZEROING;
    shoulderState = ShoulderState.ZEROING;
    pivotArmFalcon.configForwardSoftLimitEnable(false);
    pivotArmFalcon.configReverseSoftLimitEnable(false);
    fixedArmFalcon.configForwardSoftLimitEnable(false);
    fixedArmFalcon.configReverseSoftLimitEnable(false);
    enableFixedRatchet(false);
    enablePivotRatchet(false);

    pivotArmFalcon.configSupplyCurrentLimit(
        ClimbConstants.getZeroSupplyCurrentLimit(), Constants.kTalonConfigTimeout);
    fixedArmFalcon.configSupplyCurrentLimit(
        ClimbConstants.getZeroSupplyCurrentLimit(), Constants.kTalonConfigTimeout);
    pivotArmFalcon.configStatorCurrentLimit(
        ClimbConstants.getZeroStatorCurrentLimit(), Constants.kTalonConfigTimeout);
    fixedArmFalcon.configStatorCurrentLimit(
        ClimbConstants.getZeroStatorCurrentLimit(), Constants.kTalonConfigTimeout);

    openLoopPivotArm(ClimbConstants.kClimbArmsZeroSpeed);
    openLoopFixedArm(ClimbConstants.kClimbArmsZeroSpeed);
  }

  public boolean isLeftArmTouchingBar() {
    if (!leftArmHome.get()) leftHomeStableCounts++;
    else leftHomeStableCounts = 0;

    return leftHomeStableCounts >= ClimbConstants.kHomeSensorStableCounts;
  }

  public boolean isRightArmTouchingBar() {
    if (!rightArmHome.get()) rightHomeStableCounts++;
    else rightHomeStableCounts = 0;
    return rightHomeStableCounts >= ClimbConstants.kHomeSensorStableCounts;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(pivotArmFalcon);
    telemetryService.register(fixedArmFalcon);
    telemetryService.register(shoulderTalon);
  }

  private Double getLeftArmHome() {
    return leftArmHome.get() ? 1.0 : 0.0;
  }

  private Double getRightArmHome() {
    return rightArmHome.get() ? 1.0 : 0.0;
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("fixedArmSevo", () -> fixedRatchet.get()),
        new Measure("pivotArmServo", () -> pivotRatchet.get()),
        new Measure("leftArmHome", () -> getLeftArmHome()),
        new Measure("rightArmHome", () -> getRightArmHome()));
  }

  public void initiateClimb() {
    logger.info("Initiating climb sequence");
    isClimbDone = false;
    isClimbInitiated = false;

    disengageFixedRatchet();
    disengagePivotRatchet();
    currFixedArmState = FixedArmState.DISENGAGE_RATCHET;
    currPivotArmState = PivotArmState.DISENGAGE_RATCHET;
    desiredFixedArmState = FixedArmState.MID_EXT;
    desiredPivotArmState = PivotArmState.IDLE;
  }

  public void midClimb() {
    logger.info("{} -> MID_RET", currFixedArmState);

    continueToHigh = false;
    continueToTraverse = false;

    currFixedArmState = FixedArmState.MID_RET;
    openLoopFixedArm(ClimbConstants.kFixedArmRetractSpeed);
    enableFixedRatchet(true);
  }

  public void highClimb() {
    logger.info("{} -> MID_RET", currFixedArmState);

    continueToHigh = true;
    continueToTraverse = false;

    currFixedArmState = FixedArmState.MID_RET;
    openLoopFixedArm(ClimbConstants.kFixedArmRetractSpeed);
    enableFixedRatchet(true);
  }

  public void traverseClimb() {
    logger.info("{} -> MID_RET", currFixedArmState);

    continueToHigh = true;
    continueToTraverse = true;

    currFixedArmState = FixedArmState.MID_RET;
    openLoopFixedArm(ClimbConstants.kFixedArmRetractSpeed);
    enableFixedRatchet(true);
  }

  public boolean isClimbDone() {
    return isClimbDone;
  }

  public boolean isClimbInitiated() {
    return isClimbInitiated;
  }

  public void setClimbIdle() {
    currFixedArmState = FixedArmState.IDLE;
    currPivotArmState = PivotArmState.IDLE;
    shoulderState = ShoulderState.IDLE;
  }

  @Override
  public void periodic() {
    switch (currFixedArmState) {
      case IDLE:
        // Do Nothing
        break;
      case ZEROING:
        if (Math.abs(fixedArmFalcon.getSelectedSensorVelocity())
            < ClimbConstants.kZeroTargetSpeedTicksPer100ms) {
          fixedArmZeroStableCounts++;
        } else {
          fixedArmZeroStableCounts = 0;
        }

        if (fixedArmZeroStableCounts > ClimbConstants.kZeroStableCounts) {
          fixedArmFalcon.setSelectedSensorPosition(0.0);
          fixedArmFalcon.configSupplyCurrentLimit(
              ClimbConstants.getFixedArmSupplyCurrentLimit(), Constants.kTalonConfigTimeout);
          fixedArmFalcon.configStatorCurrentLimit(
              ClimbConstants.getFixedArmStatorCurrentLimit(), Constants.kTalonConfigTimeout);
          fixedArmFalcon.configForwardSoftLimitEnable(false);
          fixedArmFalcon.configReverseSoftLimitEnable(false);
          openLoopFixedArm(0.0);
          logger.info("Fixed: {} -> ZEROED");
          currFixedArmState = FixedArmState.ZEROED;
          break;
        }

        break;
      case ZEROED:
        if (!isFixedRatchetOn
            && fixedArmFalcon.getSelectedSensorPosition()
                <= ClimbConstants.kPostZeroTickArmRatchetOn) {
          enableFixedRatchet(true);
        }
        break;
      case DISENGAGE_RATCHET:
        if (disengageFixedArmRatchetTimer.hasElapsed(ClimbConstants.kDisengageRatchetServoTimer)
            && !motorOnDisengageFixedRatchet) {
          openLoopFixedArm(ClimbConstants.kDisengageRatchetSpeed);
          motorOnDisengageFixedRatchet = true;
        } else if (motorOnDisengageFixedRatchet
            && (fixedArmFalcon.getSelectedSensorPosition() - fixedArmDisengageRatchetStartTicks)
                > ClimbConstants.kDisengageRatchetTicks) {
          logger.info("Fixed: DISENGAGE_RATCHET -> {}", desiredFixedArmState);
          currFixedArmState = desiredFixedArmState;
          if (desiredFixedArmState != FixedArmState.IDLE) {
            openLoopFixedArm(
                desiredFixedArmState.isExtend
                    ? ClimbConstants.kFixedArmExtendSpeed
                    : ClimbConstants.kFixedArmRetractSpeed);
          } else openLoopFixedArm(0.0);
        }
        break;
      case MID_EXT:
        if (isFixedArmOpenLoopExtendFinished(currFixedArmState.setpoint)) {
          isClimbInitiated = true;
          openLoopFixedArm(0.0);
          currFixedArmState = FixedArmState.IDLE;
        }
        break;
      case MID_RET:
        if (isFixedArmOpenLoopRetractFinished(currFixedArmState.setpoint)) {
          if (continueToHigh) {
            logger.info("Shoulder: {} -> HIGH_PVT_BK1", shoulderState);
            shoulderState = ShoulderState.HIGH_PVT_BK1;
            rotateShoulder(shoulderState.setpoint);
            currFixedArmState = FixedArmState.IDLE;
            openLoopFixedArm(0.0);
          } else {
            logger.info("Finished Mid Climb");
            currFixedArmState = FixedArmState.IDLE;
            currPivotArmState = PivotArmState.IDLE;
            shoulderState = ShoulderState.IDLE;
            openLoopFixedArm(0.0);
            openLoopPivotArm(0.0);
            isClimbDone = true;
          }
        }
        break;
      case HIGH_EXT:
        if (isFixedArmOpenLoopExtendFinished(currFixedArmState.setpoint)) {
          logger.info("Shoulder: {} -> HIGH)PVT_BK3", shoulderState);
          shoulderState = ShoulderState.HIGH_PVT_BK3;
          rotateShoulder(shoulderState.setpoint);
          currFixedArmState = FixedArmState.IDLE;
          openLoopFixedArm(0.0);
        }
        break;
      case TVS_RET1:
        if (isFixedArmOpenLoopRetractFinished(currFixedArmState.setpoint)) {
          logger.info("Shoulder: {} -> TVS_PVT_BK1", shoulderState);
          shoulderState = ShoulderState.TVS_PVT_BK1;
          rotateShoulder(shoulderState.setpoint);
          currFixedArmState = FixedArmState.IDLE;
          openLoopFixedArm(0.0);
        }
        break;
      case TVS_EXT:
        if (isFixedArmOpenLoopExtendFinished(currFixedArmState.setpoint)) {
          logger.info("Shoulder: {} -> TVS_PVT_BK3", shoulderState);
          shoulderState = ShoulderState.TVS_PVT_BK3;
          rotateShoulder(shoulderState.setpoint);
          currFixedArmState = FixedArmState.IDLE;
          openLoopFixedArm(0.0);
        }
        break;
      case TVS_RET2:
        if (isFixedArmOpenLoopRetractFinished(currFixedArmState.setpoint)) {
          logger.info("Traverse Climb Finished");
          isClimbDone = true;
          shoulderState = ShoulderState.IDLE;
          currFixedArmState = FixedArmState.IDLE;
          currPivotArmState = PivotArmState.IDLE;
          openLoopFixedArm(0.0);
          openLoopPivotArm(0.0);
        }
        break;
    }
    switch (currPivotArmState) {
      case IDLE:
        // Do Nothing
        break;
      case ZEROING:
        if (Math.abs(pivotArmFalcon.getSelectedSensorVelocity())
            < ClimbConstants.kZeroTargetSpeedTicksPer100ms) {
          pivotArmZeroStableCounts++;
        } else {
          pivotArmZeroStableCounts = 0;
        }

        if (pivotArmZeroStableCounts > ClimbConstants.kZeroStableCounts) {
          pivotArmFalcon.setSelectedSensorPosition(0.0);
          pivotArmFalcon.configSupplyCurrentLimit(
              ClimbConstants.getPivotArmSupplyCurrentLimit(), Constants.kTalonConfigTimeout);
          pivotArmFalcon.configStatorCurrentLimit(
              ClimbConstants.getPivotArmStatorCurrentLimit(), Constants.kTalonConfigTimeout);
          pivotArmFalcon.configForwardSoftLimitEnable(false);
          pivotArmFalcon.configReverseSoftLimitEnable(false);
          logger.info("Pivot: {} -> ZEROED");
          currPivotArmState = PivotArmState.ZEROED;
          openLoopPivotArm(0.0);
          break;
        }
        break;
      case ZEROED:
        if (!isPivotRatchetOn
            && pivotArmFalcon.getSelectedSensorPosition()
                <= ClimbConstants.kPostZeroTickArmRatchetOn) enablePivotRatchet(true);
        break;
      case DISENGAGE_RATCHET:
        if (disengagePivotArmRatchetTimer.hasElapsed(ClimbConstants.kDisengageRatchetServoTimer)
            && !motorOnDisengagePivotRatchet) {
          openLoopPivotArm(ClimbConstants.kDisengageRatchetSpeed);
          motorOnDisengagePivotRatchet = true;
        } else if (motorOnDisengagePivotRatchet
            && (pivotArmFalcon.getSelectedSensorPosition() - pivotArmDisengageRatchetStartTicks)
                > ClimbConstants.kDisengageRatchetTicks) {
          logger.info("Fixed: DISENGAGE_RATCHET -> {}", desiredPivotArmState);
          currPivotArmState = desiredPivotArmState;
          if (desiredPivotArmState != PivotArmState.IDLE) {
            openLoopPivotArm(
                desiredPivotArmState.isExtend
                    ? ClimbConstants.kPivotArmExtendSpeed
                    : ClimbConstants.kPivotArmRetractSpeed);
          } else openLoopPivotArm(0.0);
        }
        break;
      case HIGH_EXT:
        if (isPivotArmOpenLoopExtendFinished(currPivotArmState.setpoint)) {
          logger.info("Shoulder: {} -> HIGH_PVT_FWD1", shoulderState);
          shoulderState = ShoulderState.HIGH_PVT_FWD1;
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
          openLoopPivotArm(0.0);
        }
        break;
      case HIGH_RET1:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          logger.info("Shoulder: {} -> HIGH_PVT_BK2", shoulderState);
          shoulderState = ShoulderState.HIGH_PVT_BK2;
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
          openLoopPivotArm(0.0);
        }
        break;
      case HIGH_RET2:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          logger.info("Shoulder: {} -> HIGH_PVT_FWD2", shoulderState);
          shoulderState = ShoulderState.HIGH_PVT_FWD2;
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
          openLoopPivotArm(0.0);
        }
        break;
      case TVS_EXT:
        if (isPivotArmOpenLoopExtendFinished(currPivotArmState.setpoint)) {
          logger.info("Shoulder: {} -> TVS_PVT_FWD", shoulderState);
          shoulderState = ShoulderState.TVS_PVT_FWD;
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
          openLoopPivotArm(0.0);
        }
        break;
      case TVS_RET1:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          logger.info("Shoulder: {} -> TVS_PVT_BCK2", shoulderState);
          shoulderState = ShoulderState.TVS_PVT_BK2;
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
          openLoopPivotArm(0.0);
        }
        break;
    }
    switch (shoulderState) {
      case IDLE:
        // Do Nothing
        break;
      case ZEROING:
        int absPos = shoulderTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
        int offset = absPos - ClimbConstants.kShoulderZeroTicks;
        shoulderTalon.setSelectedSensorPosition(offset);
        logger.info("Shoulder: {} -> ZEROED, absPos: {}, offset: {}", absPos, offset);
        shoulderState = ShoulderState.ZEROED;
        rotateShoulder(ClimbConstants.kShoulderPostZeroTicks);
        break;
      case ZEROED:
        break;
      case HIGH_PVT_BK1:
        if (isShoulderFinished()) {
          logger.info("Pivot: {} -> HIGH_EXT", currPivotArmState);
          currPivotArmState = PivotArmState.HIGH_EXT;
          openLoopPivotArm(ClimbConstants.kPivotArmExtendSpeed);
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case HIGH_PVT_FWD1:
        if (isShoulderFinished()) {
          logger.info("Pivot: {} -> HIGH_RET1", currPivotArmState);
          currPivotArmState = PivotArmState.HIGH_RET1;
          enablePivotRatchet(true);
          openLoopPivotArm(ClimbConstants.kPivotArmRetractSpeed);
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case HIGH_PVT_BK2:
        if (isShoulderFinished()) {
          logger.info("Fixed: {} -> DISENGAGE_RATCHET", currFixedArmState);
          currFixedArmState = FixedArmState.DISENGAGE_RATCHET;
          desiredFixedArmState = FixedArmState.HIGH_EXT;
          disengageFixedRatchet();
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case HIGH_PVT_BK3:
        if (isShoulderFinished()) {
          logger.info("Pivot: {} -> HIGH_RET2", currPivotArmState);
          currPivotArmState = PivotArmState.HIGH_RET2;
          openLoopPivotArm(ClimbConstants.kPivotArmRetractSpeed);
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case HIGH_PVT_FWD2:
        if (isShoulderFinished()) {
          if (continueToTraverse) {
            logger.info("Fixed: {} -> TVS_RET1", currFixedArmState);
            currFixedArmState = FixedArmState.TVS_RET1;
            enableFixedRatchet(true);
            openLoopFixedArm(ClimbConstants.kFixedArmRetractSpeed);
            shoulderState = ShoulderState.IDLE;
          } else {
            logger.info("Finished High Climb");
            currFixedArmState = FixedArmState.IDLE;
            currPivotArmState = PivotArmState.IDLE;
            shoulderState = ShoulderState.IDLE;
            openLoopPivotArm(0.0);
            openLoopFixedArm(0.0);
            isClimbDone = true;
          }
        }
        break;
      case TVS_PVT_BK1:
        if (isShoulderFinished()) {
          logger.info("Pivot: {} -> DISENGAGE_RATCHET", currPivotArmState);
          currPivotArmState = PivotArmState.DISENGAGE_RATCHET;
          desiredPivotArmState = PivotArmState.TVS_EXT;
          disengagePivotRatchet();
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case TVS_PVT_FWD:
        if (isShoulderFinished()) {
          logger.info("Pivot: {} -> TVS_RET1", currPivotArmState);
          currPivotArmState = PivotArmState.TVS_RET1;
          enablePivotRatchet(true);
          openLoopPivotArm(ClimbConstants.kPivotArmRetractSpeed);
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case TVS_PVT_BK2:
        if (isShoulderFinished()) {
          logger.info("Fixed: {} -> DISENGAGE_RATCHET", currFixedArmState);
          currFixedArmState = FixedArmState.DISENGAGE_RATCHET;
          desiredFixedArmState = FixedArmState.TVS_EXT;
          disengageFixedRatchet();
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case TVS_PVT_BK3:
        if (isShoulderFinished()) {
          logger.info("Fixed: {} -> TVS_RET2", currFixedArmState);
          currFixedArmState = FixedArmState.TVS_RET2;
          openLoopFixedArm(ClimbConstants.kFixedArmRetractSpeed);
          shoulderState = shoulderState.IDLE;
        }
        break;
    }
  }

  public enum FixedArmState {
    IDLE(0, false), // not used
    ZEROING(0, false), // not used
    ZEROED(0, false), // not used
    DISENGAGE_RATCHET(0, false), // not used
    MID_EXT(ClimbConstants.kFMidExtTicks, true),
    MID_RET(ClimbConstants.kFMidRetTicks, false),
    HIGH_EXT(ClimbConstants.kFHighExtTicks, true),
    TVS_RET1(ClimbConstants.kFTvsRet1Ticks, false),
    TVS_EXT(ClimbConstants.kFTvsExtTicks, true),
    TVS_RET2(ClimbConstants.kFTvsRet2Ticks, false);

    public final double setpoint;
    public final boolean isExtend;

    FixedArmState(double setpoint, boolean isExtend) {
      this.setpoint = setpoint;
      this.isExtend = isExtend;
    }
  }

  public enum PivotArmState {
    IDLE(0, false), // not used
    ZEROING(0, false), // not used
    ZEROED(0, false), // not used
    DISENGAGE_RATCHET(0, false), // not used
    HIGH_EXT(ClimbConstants.kPHighExtTicks, true),
    HIGH_RET1(ClimbConstants.kPHighRet1Ticks, false),
    HIGH_RET2(ClimbConstants.kPHighRet2Ticks, false),
    TVS_EXT(ClimbConstants.kPTvsExtTicks, true),
    TVS_RET1(ClimbConstants.kPTvsRetTicks, false);

    public final double setpoint;
    public final boolean isExtend;

    PivotArmState(double setpoint, boolean isExtend) {
      this.setpoint = setpoint;
      this.isExtend = isExtend;
    }
  }

  public enum ShoulderState {
    IDLE(0), // not used
    ZEROING(0), // not used
    ZEROED(0), // not used
    HIGH_PVT_BK1(ClimbConstants.kHighPvtBck1Ticks),
    HIGH_PVT_FWD1(ClimbConstants.kHighPvtFwd1Ticks),
    HIGH_PVT_BK2(ClimbConstants.kHighPvtBck2Ticks),
    HIGH_PVT_BK3(ClimbConstants.kHighPvtBck3Ticks),
    HIGH_PVT_FWD2(ClimbConstants.kHighPvtFwd2Ticks),
    TVS_PVT_BK1(ClimbConstants.kTvsPvtBck1Ticks),
    TVS_PVT_FWD(ClimbConstants.kTvsPvtFwdTicks),
    TVS_PVT_BK2(ClimbConstants.kTvsPvtBck2Ticks),
    TVS_PVT_BK3(ClimbConstants.kTvsPvtBck3Ticks);

    public final double setpoint;

    ShoulderState(double setpoint) {
      this.setpoint = setpoint;
    }
  }
}
