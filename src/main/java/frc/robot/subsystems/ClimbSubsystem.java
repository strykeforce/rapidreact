package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
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

  private double pivotArmSetPointTicks;
  private double fixedArmSetPointTicks;
  private double shoulderSetPointTicks;
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

    configTalons();
  }

  private void configTalons() {
    pivotArmFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    pivotArmFalcon.configAllSettings(
        ClimbConstants.getExtendFalconConfig(), Constants.kTalonConfigTimeout);
    pivotArmFalcon.enableVoltageCompensation(true);
    pivotArmFalcon.setNeutralMode(NeutralMode.Brake);

    fixedArmFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    fixedArmFalcon.configAllSettings(
        ClimbConstants.getExtendFalconConfig(), Constants.kTalonConfigTimeout);
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
    enableFixedRatchet(false);
    actuateFixedArm(fixedArmSetPointTicks + ClimbConstants.kDisengageFixedTicks);
  }

  public void disengagePivotRatchet() {
    enablePivotRatchet(false);
    actuatePivotArm(pivotArmSetPointTicks + ClimbConstants.kDisengagePivotTicks);
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

    shoulderTalon.configSupplyCurrentLimit(ClimbConstants.getZeroCurrentLimit());
    pivotArmFalcon.configSupplyCurrentLimit(ClimbConstants.getZeroCurrentLimit());
    fixedArmFalcon.configSupplyCurrentLimit(ClimbConstants.getZeroCurrentLimit());

    openLoopPivotArm(ClimbConstants.kClimbArmsZeroSpeed);
    openLoopFixedArm(ClimbConstants.kClimbArmsZeroSpeed);
  }

  public boolean isLeftArmTouchingBar() {
    return !leftArmHome.get();
  }

  public boolean isRightArmTouchingBar() {
    return !rightArmHome.get();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(pivotArmFalcon);
    telemetryService.register(fixedArmFalcon);
    telemetryService.register(shoulderTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public void initiateClimb() {
    logger.info("Initiating climb sequence");
    isClimbDone = false;

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
    actuateFixedArm(currFixedArmState.setpoint);
    enableFixedRatchet(true);
  }

  public void highClimb() {
    logger.info("{} -> MID_RET", currFixedArmState);

    continueToHigh = true;
    continueToTraverse = false;

    currFixedArmState = FixedArmState.MID_RET;
    actuateFixedArm(currFixedArmState.setpoint);
  }

  public void traverseClimb() {
    logger.info("{} -> MID_RET", currFixedArmState);

    continueToHigh = true;
    continueToTraverse = true;

    currFixedArmState = FixedArmState.MID_RET;
    actuateFixedArm(currFixedArmState.setpoint);
  }

  public boolean isClimbDone() {
    return isClimbDone;
  }

  public boolean isClimbInitiated() {
    return currFixedArmState == FixedArmState.MID_EXT && isFixedArmFinished();
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
          logger.info("Fixed: {} -> ZEROED");
          currFixedArmState = FixedArmState.ZEROED;
          break;
        }

        break;
      case ZEROED:
        break;
      case DISENGAGE_RATCHET:
        if (isFixedArmFinished()) {
          logger.info("Fixed: DISENGAGE_RATCHET -> {}", desiredFixedArmState);
          currFixedArmState = desiredFixedArmState;
          if (desiredFixedArmState != FixedArmState.IDLE) {
            actuateFixedArm(desiredFixedArmState.setpoint);
          }
        }
        break;
      case MID_EXT:
        // wait
        break;
      case MID_RET:
        if (isFixedArmFinished()) {
          if (continueToHigh) {
            logger.info("Shoulder: {} -> HIGH_PVT_BK1", shoulderState);
            shoulderState = ShoulderState.HIGH_PVT_BK1;
            rotateShoulder(shoulderState.setpoint);
            currFixedArmState = FixedArmState.IDLE;
          } else {
            logger.info("Finished Mid Climb");
            currFixedArmState = FixedArmState.IDLE;
            currPivotArmState = PivotArmState.IDLE;
            shoulderState = ShoulderState.IDLE;
            isClimbDone = true;
          }
        }
        break;
      case HIGH_EXT:
        if (isFixedArmFinished()) {
          logger.info("Shoulder: {} -> HIGH)PVT_BK3", shoulderState);
          shoulderState = ShoulderState.HIGH_PVT_BK3;
          rotateShoulder(shoulderState.setpoint);
          currFixedArmState = FixedArmState.IDLE;
        }
        break;
      case TVS_RET:
        if (isFixedArmFinished()) {
          logger.info("Shoulder: {} -> TVS_PVT_BK1", shoulderState);
          shoulderState = ShoulderState.TVS_PVT_BK1;
          rotateShoulder(shoulderState.setpoint);
          currFixedArmState = FixedArmState.IDLE;
        }
        break;
      case TVS_EXT:
        if (isFixedArmFinished()) {
          logger.info("Shoulder: {} -> TVS_PVT_BK3", shoulderState);
          shoulderState = ShoulderState.TVS_PVT_BK3;
          rotateShoulder(shoulderState.setpoint);
          currFixedArmState = FixedArmState.IDLE;
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
          logger.info("Pivot: {} -> ZEROED");
          currPivotArmState = PivotArmState.ZEROED;
          break;
        }
        break;
      case ZEROED:
        break;
      case DISENGAGE_RATCHET:
        if (isPivotArmFinished()) {
          logger.info("Pivot: DISENGAGE_RATCHET -> {}", desiredPivotArmState);
          currPivotArmState = desiredPivotArmState;
          if (desiredPivotArmState != PivotArmState.IDLE) {
            actuatePivotArm(desiredPivotArmState.setpoint);
          }
        }
        break;
      case HIGH_EXT:
        if (isPivotArmFinished()) {
          logger.info("Shoulder: {} -> HIGH_PVT_FWD1", shoulderState);
          shoulderState = ShoulderState.HIGH_PVT_FWD1;
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
        }
        break;
      case HIGH_RET1:
        if (isPivotArmFinished()) {
          logger.info("Shoulder: {} -> HIGH_PVT_BK2", shoulderState);
          shoulderState = ShoulderState.HIGH_PVT_BK2;
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
        }
        break;
      case HIGH_RET2:
        if (isPivotArmFinished()) {
          logger.info("Shoulder: {} -> HIGH_PVT_FWD2", shoulderState);
          shoulderState = ShoulderState.HIGH_PVT_FWD2;
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
        }
        break;
      case TVS_EXT:
        if (isPivotArmFinished()) {
          logger.info("Shoulder: {} -> TVS_PVT_FWD", shoulderState);
          shoulderState = ShoulderState.TVS_PVT_FWD;
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
        }
        break;
      case TVS_RET:
        if (isPivotArmFinished()) {
          logger.info("Shoulder: {} -> TVS_PVT_BCK2", shoulderState);
          shoulderState = ShoulderState.TVS_PVT_BK2;
          rotateShoulder(shoulderState.setpoint);
          currPivotArmState = PivotArmState.IDLE;
        }
        break;
    }
    switch (shoulderState) {
      case IDLE:
        // Do Nothing
        break;
      case ZEROING:
        int absPos = shoulderTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
        int offset = ClimbConstants.kShoulderZeroTicks - absPos;
        shoulderTalon.setSelectedSensorPosition(offset);
        logger.info("Shoulder: {} -> ZEROED, absPos: {}, offset: {}", absPos, offset);
        shoulderState = ShoulderState.ZEROED;
        break;
      case ZEROED:
        break;
      case HIGH_PVT_BK1:
        if (isShoulderFinished()) {
          logger.info("Pivot: {} -> HIGH_EXT", currPivotArmState);
          currPivotArmState = PivotArmState.HIGH_EXT;
          actuatePivotArm(currPivotArmState.setpoint);
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case HIGH_PVT_FWD1:
        if (isShoulderFinished()) {
          logger.info("Pivot: {} -> HIGH_RET1", currPivotArmState);
          currPivotArmState = PivotArmState.HIGH_RET1;
          enablePivotRatchet(true);
          actuatePivotArm(currPivotArmState.setpoint);
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
          actuatePivotArm(currPivotArmState.setpoint);
          shoulderState = ShoulderState.IDLE;
        }
        break;
      case HIGH_PVT_FWD2:
        if (isShoulderFinished()) {
          if (continueToTraverse) {
            logger.info("Fixed: {} -> TVS_RET", currFixedArmState);
            currFixedArmState = FixedArmState.TVS_RET;
            enableFixedRatchet(true);
            actuateFixedArm(currFixedArmState.setpoint);
            shoulderState = ShoulderState.IDLE;
          } else {
            logger.info("Finished High Climb");
            currFixedArmState = FixedArmState.IDLE;
            currPivotArmState = PivotArmState.IDLE;
            shoulderState = ShoulderState.IDLE;
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
          logger.info("Pivot: {} -> TVS_RET", currPivotArmState);
          currPivotArmState = PivotArmState.TVS_RET;
          enablePivotRatchet(true);
          actuatePivotArm(currPivotArmState.setpoint);
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
          logger.info("Finished Traverse Climb");
          shoulderState = ShoulderState.IDLE;
          currFixedArmState = FixedArmState.IDLE;
          currPivotArmState = PivotArmState.IDLE;
          isClimbDone = true;
        }
        break;
    }
  }

  public enum FixedArmState {
    IDLE(0),
    ZEROING(0),
    ZEROED(0),
    DISENGAGE_RATCHET(0),
    MID_EXT(ClimbConstants.kFMidExtTicks),
    MID_RET(ClimbConstants.kFMidRetTicks),
    HIGH_EXT(ClimbConstants.kFHighExtTicks),
    TVS_RET(ClimbConstants.kFTvsRetTicks),
    TVS_EXT(ClimbConstants.kFTvsExtTicks);

    public final double setpoint;

    FixedArmState(double setpoint) {
      this.setpoint = setpoint;
    }
  }

  public enum PivotArmState {
    IDLE(0),
    ZEROING(0),
    ZEROED(0),
    DISENGAGE_RATCHET(0),
    HIGH_EXT(ClimbConstants.kPHighExtTicks),
    HIGH_RET1(ClimbConstants.kPHighRet1Ticks),
    HIGH_RET2(ClimbConstants.kPHighRet2Ticks),
    TVS_EXT(ClimbConstants.kPTvsExtTicks),
    TVS_RET(ClimbConstants.kPTvsRetTicks);

    public final double setpoint;

    PivotArmState(double setpoint) {
      this.setpoint = setpoint;
    }
  }

  public enum ShoulderState {
    IDLE(0),
    ZEROING(0),
    ZEROED(0),
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
