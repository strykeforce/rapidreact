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
  private final Timer climbDelayTimer = new Timer();

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

  private double climbStateCounter = 0.0;

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

  public void setShoulderCruise(double cruiseVel) {
    logger.info("Set Shoulder Cruise Vel to: {}", cruiseVel);
    shoulderTalon.configMotionCruiseVelocity(cruiseVel, Constants.kTalonConfigTimeout);
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
    logger.info("Setting Pivot Ratchet to {}", isPivotRatchetOn);
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

  private Double getLeftArmHome() {
    return leftArmHome.get() ? 1.0 : 0.0;
  }

  private Double getRightArmHome() {
    return rightArmHome.get() ? 1.0 : 0.0;
  }

  public void initiateClimb() {
    logger.info("Initiating climb sequence");
    isClimbDone = false;
    isClimbInitiated = false;
    climbStateCounter = 0;

    disengageFixedRatchet();
    disengagePivotRatchet();
    currFixedArmState = FixedArmState.DISENGAGE_RATCHET;
    currPivotArmState = PivotArmState.DISENGAGE_RATCHET;
    desiredFixedArmState = FixedArmState.MID_EXT;
    desiredPivotArmState = PivotArmState.MID_EXT;
  }

  public void midClimb() {
    logger.info("Starting Mid Climb: {} -> MID_RET_ST1", currPivotArmState);

    continueToHigh = false;
    continueToTraverse = false;

    currPivotArmState = PivotArmState.MID_RET_ST1;
    openLoopPivotArm(currPivotArmState.speed);
    enablePivotRatchet(true);
  }

  public void highClimb() {
    logger.info("Starting High Climb: {} -> MID_RET_ST1", currPivotArmState);

    continueToHigh = true;
    continueToTraverse = false;

    currPivotArmState = PivotArmState.MID_RET_ST1;
    openLoopPivotArm(currPivotArmState.speed);
    enablePivotRatchet(true);
  }

  public void traverseClimb() {
    logger.info("Starting Traverse Climb: {} -> MID_RET_ST1", currPivotArmState);

    continueToHigh = true;
    continueToTraverse = true;

    currPivotArmState = PivotArmState.MID_RET_ST1;
    openLoopPivotArm(currPivotArmState.speed);
    enablePivotRatchet(true);
  }

  public void emergencyStopClimb() {
    logger.info(
        "Emergency Stop Climb Sequence: Shoulder: {}, Fixed: {}, Pivot: {}",
        shoulderState,
        currFixedArmState,
        currPivotArmState);
    currFixedArmState = FixedArmState.IDLE;
    currPivotArmState = PivotArmState.IDLE;
    shoulderState = ShoulderState.IDLE;
    openLoopFixedArm(0.0);
    openLoopPivotArm(0.0);
    rotateShoulder(shoulderTalon.getSelectedSensorPosition());
    enableFixedRatchet(true);
    enablePivotRatchet(true);
    isClimbDone = true;
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
          climbStateCounter++;
          currFixedArmState = desiredFixedArmState;
          if (desiredFixedArmState != FixedArmState.IDLE) {
            openLoopFixedArm(desiredFixedArmState.speed);
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
      case HIGH_RET_ST1:
        if (isFixedArmOpenLoopRetractFinished(currFixedArmState.setpoint)) {
          // logger.info("Fixed: {} -> HIGH_RET_ST2", currFixedArmState);
          // currFixedArmState = FixedArmState.HIGH_RET_ST2;
          // openLoopFixedArm(currFixedArmState.speed);
          // climbStateCounter++;
          logger.info("Fixed: {} -> IDLE", currFixedArmState);
          currFixedArmState = FixedArmState.IDLE;
          openLoopFixedArm(0.0);
        }
        break;
      case HIGH_RET_ST2:
        if (isFixedArmOpenLoopRetractFinished(currFixedArmState.setpoint)) {
          if (continueToTraverse) {
            logger.info("TVS DELAY: {}", ClimbConstants.kTvsDelay);
            currFixedArmState = FixedArmState.TVS_DELAY;
            climbDelayTimer.reset();
            climbDelayTimer.start();
            openLoopFixedArm(0.0);
            climbStateCounter++;
          } else {
            logger.info("Finished high climb");
            shoulderState = ShoulderState.IDLE;
            currFixedArmState = FixedArmState.IDLE;
            currPivotArmState = PivotArmState.IDLE;
            openLoopFixedArm(0.0);
            openLoopPivotArm(0.0);
            isClimbDone = true;
          }
        }
        break;
      case TVS_DELAY:
        if (climbDelayTimer.hasElapsed(ClimbConstants.kTvsDelay)) {
          logger.info(
              "Shoulder: {} -> HIGH_PVT_BK2, Pivot: {} -> HIGH_RET2",
              shoulderState,
              currPivotArmState);
          shoulderState = ShoulderState.HIGH_PVT_BK2;
          currPivotArmState = PivotArmState.HIGH_RET2;
          setShoulderCruise(shoulderState.cruiseVel);
          rotateShoulder(shoulderState.setpoint);
          openLoopPivotArm(currPivotArmState.speed);
          currFixedArmState = FixedArmState.IDLE;
          openLoopFixedArm(0.0);
          climbStateCounter++;
        }
        break;
      case TVS_RET_ST1:
        if (isFixedArmOpenLoopRetractFinished(currFixedArmState.setpoint)) {
          logger.info("Fixed: {} -> TVS_RET_ST2", currFixedArmState);
          currFixedArmState = FixedArmState.TVS_RET_ST2;
          openLoopFixedArm(currFixedArmState.speed);
          climbStateCounter++;
        }
        break;
      case TVS_RET_ST2:
        if (isFixedArmOpenLoopRetractFinished(currFixedArmState.setpoint)) {
          logger.info("Shoulder: {} -> TVS_PVT_BK", shoulderState);
          shoulderState = ShoulderState.TVS_PVT_BK;
          setShoulderCruise(shoulderState.cruiseVel);
          rotateShoulder(shoulderState.setpoint);
          currFixedArmState = FixedArmState.IDLE;
          openLoopFixedArm(0.0);
          climbStateCounter++;
        }
        break;
      case TVS_EXT_ST1:
        if (isFixedArmOpenLoopExtendFinished(currFixedArmState.setpoint)) {
          logger.info(
              "Fixed: {} -> TVS_EXT_ST2, Shoulder: {} -> TVS_PVT_FWD2",
              currFixedArmState,
              shoulderState);
          currFixedArmState = FixedArmState.TVS_EXT_ST2;
          openLoopFixedArm(currFixedArmState.speed);
          shoulderState = ShoulderState.TVS_PVT_FWD2;
          setShoulderCruise(shoulderState.cruiseVel);
          rotateShoulder(shoulderState.setpoint);
          climbStateCounter++;
        }
        break;
      case TVS_EXT_ST2:
        if (isFixedArmOpenLoopExtendFinished(currFixedArmState.setpoint)) {
          // logger.info("Fixed: {} -> IDLE", currFixedArmState);
          // currFixedArmState = FixedArmState.IDLE;
          // openLoopFixedArm(0.0);
          logger.info("Finished traverse climb");
          shoulderState = ShoulderState.IDLE;
          currFixedArmState = FixedArmState.IDLE;
          currPivotArmState = PivotArmState.IDLE;
          openLoopFixedArm(0.0);
          openLoopPivotArm(0.0);
          isClimbDone = true;
        }
        break;
      case MID_FIN_RET:
        if (isFixedArmOpenLoopRetractFinished(currFixedArmState.setpoint)) {
          logger.info("Finished mid climb");
          shoulderState = ShoulderState.IDLE;
          currFixedArmState = FixedArmState.IDLE;
          currPivotArmState = PivotArmState.IDLE;
          openLoopFixedArm(0.0);
          openLoopPivotArm(0.0);
          isClimbDone = true;
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
          logger.info("Pivot: DISENGAGE_RATCHET -> {}", desiredPivotArmState);
          climbStateCounter++;
          currPivotArmState = desiredPivotArmState;
          if (desiredPivotArmState != PivotArmState.IDLE) {
            openLoopPivotArm(desiredPivotArmState.speed);
          } else openLoopPivotArm(0.0);
        }
        break;
      case MID_EXT:
        if (isPivotArmOpenLoopExtendFinished(currPivotArmState.setpoint)) {
          openLoopPivotArm(0.0);
          currPivotArmState = PivotArmState.IDLE;
        }
        break;
      case MID_RET_ST1:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          logger.info("Pivot: {} -> MID_RET_ST2", currPivotArmState);
          currPivotArmState = PivotArmState.MID_RET_ST2;
          openLoopPivotArm(currPivotArmState.speed);
          climbStateCounter++;
        }
        break;
      case MID_RET_ST2:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          logger.info("Pivot: {} -> MID_RET_ST3", currPivotArmState);
          currPivotArmState = PivotArmState.MID_RET_ST3;
          openLoopPivotArm(currPivotArmState.speed);
          climbStateCounter++;
          if (continueToHigh) {
            logger.info("Shoulder: {} -> HIGH_PVT_FWD", shoulderState);
            shoulderState = ShoulderState.HIGH_PVT_FWD;
            setShoulderCruise(shoulderState.cruiseVel);
            rotateShoulder(shoulderState.setpoint);
          }
        }
        break;
      case MID_RET_ST3:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          logger.info("Pivot: {} -> MID_RET_ST4", currPivotArmState);
          currPivotArmState = PivotArmState.MID_RET_ST4;
          openLoopPivotArm(currPivotArmState.speed);
          climbStateCounter++;
        }
        break;
      case MID_RET_ST4:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          if (continueToHigh) {
            logger.info("Pivot: {} -> IDLE", currPivotArmState);
            currPivotArmState = PivotArmState.IDLE;
            openLoopPivotArm(0.0);
          } else {
            logger.info(
                "Fixed: {} -> MID_FIN_RET, Shoulder: {} -> MID_FIN_PVT_BK",
                currFixedArmState,
                shoulderState);
            currFixedArmState = FixedArmState.MID_FIN_RET;
            openLoopFixedArm(currFixedArmState.speed);
            enableFixedRatchet(true);
            shoulderState = ShoulderState.MID_FIN_PVT_Bk;
            setShoulderCruise(shoulderState.cruiseVel);
            rotateShoulder(shoulderState.setpoint);
            currPivotArmState = PivotArmState.IDLE;
            openLoopPivotArm(0.0);
            climbStateCounter++;
          }
        }
        break;
      case HIGH_RET_ST1:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          logger.info("Pivot: {} -> HIGH_RET_ST2", currPivotArmState);
          currPivotArmState = PivotArmState.HIGH_RET_ST2;
          openLoopPivotArm(currPivotArmState.speed);
          climbStateCounter++;
        }
        break;
      case HIGH_RET_ST2:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          logger.info("Shoulder: {} -> HIGH_PVT_BK1", shoulderState);
          shoulderState = ShoulderState.HIGH_PVT_BK1;
          setShoulderCruise(shoulderState.cruiseVel);
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
          openLoopPivotArm(0.0);
          climbStateCounter++;
        }
        break;
      case HIGH_EXT_ST1:
        if (isPivotArmOpenLoopExtendFinished(currPivotArmState.setpoint)) {
          logger.info("Pivot: {} -> HIGH_EXT_ST2", currPivotArmState);
          currPivotArmState = PivotArmState.HIGH_EXT_ST2;
          openLoopPivotArm(currPivotArmState.speed);
          climbStateCounter++;
        }
        break;
      case HIGH_EXT_ST2:
        if (isPivotArmOpenLoopExtendFinished(currPivotArmState.setpoint)) {
          logger.info("Pivot: {} -> HIGH_EXT_ST3", currPivotArmState);
          currPivotArmState = PivotArmState.HIGH_EXT_ST3;
          openLoopPivotArm(currPivotArmState.speed);
          climbStateCounter++;
        }
        break;
      case HIGH_EXT_ST3:
        if (isPivotArmOpenLoopExtendFinished(currPivotArmState.setpoint)) {
          logger.info("Fixed: {} -> HIGH_RET_ST2", currFixedArmState);
          currFixedArmState = FixedArmState.HIGH_RET_ST2;
          enablePivotRatchet(true);
          openLoopFixedArm(currFixedArmState.speed);
          openLoopPivotArm(0.0);
          currPivotArmState = PivotArmState.IDLE;
          climbStateCounter++;
        }
        break;
      case HIGH_RET2:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          logger.info("Pivot: {} -> IDLE", currPivotArmState);
          currPivotArmState = PivotArmState.IDLE;
          openLoopPivotArm(0.0);
        }
        break;
      case TVS_EXT:
        if (isPivotArmOpenLoopExtendFinished(currPivotArmState.setpoint)) {
          // logger.info("Fixed: {} -> TVS_RET_ST1", currFixedArmState);
          // currFixedArmState = FixedArmState.TVS_RET_ST1;
          // openLoopFixedArm(currFixedArmState.speed);
          // currPivotArmState = PivotArmState.IDLE;
          // openLoopPivotArm(0.0);
          // climbStateCounter++;
          logger.info("Pivot: {} -> IDLE", currPivotArmState);
          currPivotArmState = PivotArmState.IDLE;
          openLoopPivotArm(0.0);
        }
        break;
      case TVS_RET_ST1:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          logger.info("Pivot: {} -> TVS_RET_ST2", currPivotArmState);
          currPivotArmState = PivotArmState.TVS_RET_ST2;
          openLoopPivotArm(currPivotArmState.speed);
          climbStateCounter++;
        }
        break;
      case TVS_RET_ST2:
        if (isPivotArmOpenLoopRetractFinished(currPivotArmState.setpoint)) {
          logger.info("Shoulder: {} -> TVS_PVT_FWD1", shoulderState);
          shoulderState = ShoulderState.TVS_PVT_FWD1;
          setShoulderCruise(shoulderState.cruiseVel);
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
          openLoopPivotArm(0.0);
          climbStateCounter++;
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
      case HIGH_PVT_FWD:
        if (isShoulderFinished()) {
          logger.info("Pivot: {} -> HIGH_RET_ST1", currPivotArmState);
          currPivotArmState = PivotArmState.HIGH_RET_ST1;
          climbStateCounter++;
          openLoopPivotArm(currPivotArmState.speed);
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case HIGH_PVT_BK1:
        if (isShoulderFinished()) {
          logger.info(
              "Pivot: {} -> DISENGAGE_RATCHET, Fixed: {} -> HIGH_RET_ST1",
              currPivotArmState,
              currFixedArmState);
          currPivotArmState = PivotArmState.DISENGAGE_RATCHET;
          desiredPivotArmState = PivotArmState.HIGH_EXT_ST1;
          currFixedArmState = FixedArmState.HIGH_RET_ST1;
          enableFixedRatchet(true);
          openLoopFixedArm(currFixedArmState.speed);
          disengagePivotRatchet();
          shoulderState = ShoulderState.IDLE;
          climbStateCounter++;
        }
        break;
      case HIGH_PVT_BK2:
        if (isShoulderFinished()) {
          logger.info(
              "Pivot: {} -> DISENGAGE_RATCHET, Fixed: {} -> TVS_RET_ST1",
              currPivotArmState,
              currFixedArmState);
          currPivotArmState = PivotArmState.DISENGAGE_RATCHET;
          desiredPivotArmState = PivotArmState.TVS_EXT;
          disengagePivotRatchet();
          currFixedArmState = FixedArmState.TVS_RET_ST1;
          openLoopFixedArm(currFixedArmState.speed);
          climbStateCounter++;
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case TVS_PVT_BK:
        if (isShoulderFinished()) {
          logger.info("Pivot: {} -> TVS_RET_ST1", currPivotArmState);
          currPivotArmState = PivotArmState.TVS_RET_ST1;
          openLoopPivotArm(currPivotArmState.speed);
          shoulderState = ShoulderState.IDLE;
          climbStateCounter++;
          enablePivotRatchet(true);
        }
        break;
      case TVS_PVT_FWD1:
        if (isShoulderFinished()) {
          logger.info("Fixed: {} -> DISENGAGE_RATCHET", currFixedArmState);
          currFixedArmState = FixedArmState.DISENGAGE_RATCHET;
          desiredFixedArmState = FixedArmState.TVS_EXT_ST1;
          disengageFixedRatchet();
          shoulderState = ShoulderState.IDLE;
          climbStateCounter++;
        }
        break;
      case TVS_PVT_FWD2:
        if (isShoulderFinished()) {
          logger.info("Shoulder: {} -> IDLE", shoulderState);
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case MID_FIN_PVT_Bk:
        if (isShoulderFinished()) {
          logger.info("Shoulder: {} -> IDLE", shoulderState);
          shoulderState = ShoulderState.IDLE;
        }
        break;
    }
  }

  public enum FixedArmState {
    IDLE(0, false, 0), // not used
    ZEROING(0, false, 0), // not used
    ZEROED(0, false, 0), // not used
    DISENGAGE_RATCHET(0, false, 0), // not used
    MID_EXT(ClimbConstants.kFMidExtTicks, true, ClimbConstants.kFMidExtSpeed),
    HIGH_RET_ST1(ClimbConstants.kFHighRetST1Ticks, false, ClimbConstants.kFHighRetST1Speed),
    HIGH_RET_ST2(ClimbConstants.kFHighRetST2Ticks, false, ClimbConstants.kFHighRetST2Speed),
    TVS_DELAY(0, false, 0), // not used
    TVS_RET_ST1(ClimbConstants.kFTvsRetST1Ticks, false, ClimbConstants.kFTvsRetST1Speed),
    TVS_RET_ST2(ClimbConstants.kFTvsRetST2Ticks, false, ClimbConstants.kFTvsRetST2Speed),
    TVS_EXT_ST1(ClimbConstants.kFTvsExtST1Ticks, true, ClimbConstants.kFTvsExtST1Speed),
    TVS_EXT_ST2(ClimbConstants.kFTvsExtST2Ticks, true, ClimbConstants.kFTvsExtST2Speed),
    MID_FIN_RET(ClimbConstants.kFMidFinRetTicks, false, ClimbConstants.kFMidFinRetSpeed);

    public final double setpoint;
    public final boolean isExtend;
    public final double speed;

    FixedArmState(double setpoint, boolean isExtend, double speed) {
      this.setpoint = setpoint;
      this.isExtend = isExtend;
      this.speed = speed;
    }
  }

  public enum PivotArmState {
    IDLE(0, false, 0), // not used
    ZEROING(0, false, 0), // not used
    ZEROED(0, false, 0), // not used
    DISENGAGE_RATCHET(0, false, 0), // not used
    MID_EXT(ClimbConstants.kPMidExtTicks, true, ClimbConstants.kPMidExtSpeed),
    MID_RET_ST1(ClimbConstants.kPMidRetST1Ticks, false, ClimbConstants.kPMidRetST1Speed),
    MID_RET_ST2(ClimbConstants.kPMidRetST2Ticks, false, ClimbConstants.kPMidRetST2Speed),
    MID_RET_ST3(ClimbConstants.kPMidRetST3Ticks, false, ClimbConstants.kPMidRetST3Speed),
    MID_RET_ST4(ClimbConstants.kPMidRetST4Ticks, false, ClimbConstants.kPMidRetST4Speed),
    HIGH_RET_ST1(ClimbConstants.kPHighRetST1Ticks, false, ClimbConstants.kPHighRetST1Speed),
    HIGH_RET_ST2(ClimbConstants.kPHighRetST2Ticks, false, ClimbConstants.kPHighRetST2Speed),
    HIGH_EXT_ST1(ClimbConstants.kPHighExtST1Ticks, true, ClimbConstants.kPHighExtST1Speed),
    HIGH_EXT_ST2(ClimbConstants.kPHighExtST2Ticks, true, ClimbConstants.kPHighExtST2Speed),
    HIGH_EXT_ST3(ClimbConstants.kPHighExtST3Ticks, true, ClimbConstants.kPHighExtST3Speed),
    HIGH_RET2(ClimbConstants.kPHighRet2Ticks, false, ClimbConstants.kPHighRet2Speed),
    TVS_EXT(ClimbConstants.kPTvsExtTicks, true, ClimbConstants.kPTvsExtSpeed),
    TVS_RET_ST1(ClimbConstants.kPTvsRetST1Ticks, false, ClimbConstants.kPTvsRetST1Speed),
    TVS_RET_ST2(ClimbConstants.kPTvsRetST2Ticks, false, ClimbConstants.kPTvsRetST2Speed);

    public final double setpoint;
    public final boolean isExtend;
    public final double speed;

    PivotArmState(double setpoint, boolean isExtend, double speed) {
      this.setpoint = setpoint;
      this.isExtend = isExtend;
      this.speed = speed;
    }
  }

  public enum ShoulderState {
    IDLE(0, ClimbConstants.kShoulderCriuiseVelDefault), // not used
    ZEROING(0, ClimbConstants.kShoulderCriuiseVelDefault), // not used
    ZEROED(0, ClimbConstants.kShoulderCriuiseVelDefault), // not used
    HIGH_PVT_FWD(ClimbConstants.kHighPvtFwdTicks, ClimbConstants.kHighPvtFwdSpeed),
    HIGH_PVT_BK1(ClimbConstants.kHighPvtBk1Ticks, ClimbConstants.kHighPvtBk1Speed),
    HIGH_PVT_BK2(ClimbConstants.kHighPvtBk2Ticks, ClimbConstants.kHighPvtBk2Speed),
    TVS_PVT_BK(ClimbConstants.kTvsPvtBkTicks, ClimbConstants.kTvsPvtBkSpeed),
    TVS_PVT_FWD1(ClimbConstants.kTvsPvtFwd1Ticks, ClimbConstants.kTvsPvtFwd1Speed),
    TVS_PVT_FWD2(ClimbConstants.kTvsPvtFwd2Ticks, ClimbConstants.kTvsPvtFwd2Speed),
    MID_FIN_PVT_Bk(ClimbConstants.kMidFinPvtBkTicks, ClimbConstants.kMidFinPvtBkSpeed);

    public final double setpoint;
    public final double cruiseVel;

    ShoulderState(double setpoint, double cruiseVel) {
      this.setpoint = setpoint;
      this.cruiseVel = cruiseVel;
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("fixedArmSevo", () -> fixedRatchet.get()),
        new Measure("pivotArmServo", () -> pivotRatchet.get()),
        new Measure("leftArmHome", () -> getLeftArmHome()),
        new Measure("rightArmHome", () -> getRightArmHome()),
        new Measure("climbStateCounter", () -> climbStateCounter),
        new Measure("fixedArmTargetTicks", () -> currFixedArmState.setpoint),
        new Measure("pivotArmTargetTicks", () -> currPivotArmState.setpoint),
        new Measure("fixedArmState", currFixedArmState::ordinal),
        new Measure("pivotArmState", currPivotArmState::ordinal),
        new Measure("shoulderState", shoulderState::ordinal));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(pivotArmFalcon);
    telemetryService.register(fixedArmFalcon);
    telemetryService.register(shoulderTalon);
  }
}
