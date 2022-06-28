package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import java.util.Set;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.deadeye.Deadeye;
import org.strykeforce.deadeye.TargetDataListener;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem
    implements TargetDataListener<HubTargetData> {

  private final Deadeye<HubTargetData> deadeye;
  private volatile HubTargetData targetData = new HubTargetData();
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private int pixelWidthStableCount = 0;
  private int previousPixelWidth = 0;
  private int numOfSerialChanges = 0;
  private int lastSerialNum = -1;
  private Timer visionCheckTime = new Timer();
  private boolean isVisionWorking = true;
  private final DriveSubsystem driveSubsystem;
  private TurretSubsystem turretSubsystem;
  private CircularBuffer gyroBuffer;
  private CircularBuffer turretBuffer;
  private CircularBuffer timestampBuffer;
  private CircularBuffer temp = new CircularBuffer(10);
  private CircularBuffer temp2 = new CircularBuffer(10);
  private CircularBuffer temp3 = new CircularBuffer(10);
  private boolean canFillBuffers = false;

  public VisionSubsystem(DriveSubsystem driveSubsystem) {
    timestampBuffer = new CircularBuffer(VisionConstants.kCircularBufferSize);
    turretBuffer = new CircularBuffer(VisionConstants.kCircularBufferSize);
    gyroBuffer = new CircularBuffer(VisionConstants.kCircularBufferSize);
    this.driveSubsystem = driveSubsystem;
    visionCheckTime.reset();
    visionCheckTime.start();
    NetworkTableInstance networkTableInstance = NetworkTableInstance.create();
    networkTableInstance.startClient("10.27.67.10");
    deadeye = new Deadeye<>("A0", HubTargetData.class, networkTableInstance);
    deadeye.setTargetDataListener(this);
    HubTargetData.kFrameCenter = deadeye.getCapture().width / 2;
    HubTargetData.kFrameVerticalCenter = deadeye.getCapture().height / 2;
  }

  public void setTurretSubsystem(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void onTargetData(HubTargetData targetData) {
    this.targetData = targetData;
    if (canFillBuffers) fillBuffers();
  }

  public HubTargetData getTargetData() {
    return targetData;
  }

  public void turnOnDeadeye() {
    deadeye.setEnabled(true);
    logger.info("Vision is turned on");
  }

  public void turnOffDeadeye() {
    deadeye.setEnabled(false);
    logger.info("Vision is turned off");
  }

  public void enable() {
    deadeye.setLightEnabled(true);
    logger.info("enabled vision system");
  }

  public void disable() {
    deadeye.setLightEnabled(false);
    logger.info("disabled vision system");
  }

  @Override
  public @NotNull Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Error Pixels", this::getErrorPixels),
        new Measure("Error Pixels Jetson", this::getErrorPixelsJetson),
        new Measure("Error Radians", this::getErrorRadians),
        new Measure("Error Degrees", this::getErrorDegrees),
        new Measure("Target Data Valid", this::getValid),
        new Measure("Test Pixel Width", this::getTargetsDistancePixel),
        new Measure("Test Ground Distance", this::getTargetsDistanceGround),
        new Measure("Is Even Num Of Targets", () -> isNumTargetsEven() ? 1.0 : 0.0),
        new Measure("Vertical Pixel Height", this::getVerticalPixelHeight),
        new Measure("Vertical Radian Offset", this::getVerticalOffsetRadians),
        new Measure("Target Data SN", () -> targetData.serial),
        new Measure("Target Data SN", () -> targetData.serial),
        new Measure("Ranging Valid", this::getRangingValid));
  }

  // these private getters are for the grapher and prevent a data race that can occur if targetData
  // updates after isValid() and before getErrorXXX()
  private boolean isNumTargetsEven() {
    var td = targetData;
    return td.isValid() ? td.isNumTargetsEven() : false;
  }

  private double getVerticalPixelHeight() {
    var td = targetData;
    return td.isValid() ? td.getVerticalPixelHeight() : 2767.0;
  }

  private double getErrorPixels() {
    var td = targetData;
    return td.isValid() ? td.getErrorPixels() : 2767.0;
  }

  private double getErrorPixelsJetson() {
    var td = targetData;
    return td.isValid() ? td.getErrorPixelsJetson() : 2767.0;
  }

  private double getErrorRadians() {
    var td = targetData;
    return td.isValid() ? td.getErrorRadians() : 2767.0;
  }

  private double getErrorDegrees() {
    var td = targetData;
    return td.isValid() ? Math.toDegrees(td.getErrorRadians()) : 2767.0;
  }

  private double getVerticalOffsetRadians() {
    var td = targetData;
    return td.isValid() ? targetData.getVerticalOffsetRadians() : 2767.0;
  }

  // not used
  public double getTargetsDistancePixel() {
    var td = targetData;
    return td.isRangingValid() ? td.testGetTargetsPixelWidth() : 2767.0;
  }

  public double getTargetsDistanceGround() {
    var td = targetData;
    return td.isRangingValid() ? td.getGroundDistance() : 2767.0;
  }

  public int getTargetPixelWidth() {
    var td = targetData;
    return td.testGetTargetsPixelWidth();
  }

  public boolean isValid() {
    return targetData.isValid();
  }

  public boolean isRangingValid() {
    return targetData.isRangingValid();
  }

  private double getValid() {
    var td = targetData;
    return td.isValid() ? 1.0 : 0.0;
  }

  private double getRangingValid() {
    var td = targetData;
    return td.isRangingValid() ? 1.0 : 0.0;
  }

  public Pose2d getVisionOdometry(
      Rotation2d turretAngle, Rotation2d gyroAngle, double distanceInches) {
    if (!isRangingValid()) {
      logger.info("Vision Odom: not valid -> keep current odom");
      return driveSubsystem.getPoseMeters();
    }

    Rotation2d errorRadians = new Rotation2d(getErrorRadians());
    Rotation2d calcAngle =
        turretAngle.plus(gyroAngle).plus(TurretConstants.kTurretRobotOffset).minus(errorRadians);
    double distanceMeters =
        Units.inchesToMeters(distanceInches) + VisionConstants.kLookupTableToLensOffset;
    double x, y;
    y =
        Math.abs(
            -TurretConstants.kHubPositionMeters.getY()
                + distanceMeters * Math.sin(calcAngle.getRadians())); // -4.121
    x =
        Math.abs(
            -TurretConstants.kHubPositionMeters.getX()
                + distanceMeters * Math.cos(calcAngle.getRadians())); // -8.23
    logger.info(
        "VISIONODOM: turretAngle: {}, gyroAngle: {}, calcAngle: {}, errorRadians: {}, distance: {}, X: {}, Y: {}, Odometry: {}",
        turretAngle,
        gyroAngle,
        calcAngle,
        errorRadians,
        distanceMeters,
        x,
        y,
        driveSubsystem.getPoseMeters());
    return new Pose2d(x, y, gyroAngle);
  }

  public TimestampedPose odomNewPoseViaVision(double distanceInches) {
    Rotation2d gyroAngle = new Rotation2d(gyroBuffer.get(VisionConstants.kBufferLookupOffset));
    Rotation2d turretAngle = new Rotation2d(turretBuffer.get(VisionConstants.kBufferLookupOffset));
    double timestamp = timestampBuffer.get(VisionConstants.kBufferLookupOffset);

    Pose2d odomPose = getVisionOdometry(turretAngle, gyroAngle, distanceInches);
    return new TimestampedPose((long) timestamp, odomPose);
  }

  public void fillBuffers() {
    gyroBuffer.addFirst(driveSubsystem.getGyroRotation2d().getRadians());
    turretBuffer.addFirst(turretSubsystem.getTurretRotation2d().getRadians());
    timestampBuffer.addFirst(RobotController.getFPGATime());
  }

  public void setFillBuffers(boolean set) {
    canFillBuffers = set;
  }

  public boolean isPixelWidthStable() {
    if (!isRangingValid()) {
      return false;
    }
    var pixelWidth = getTargetPixelWidth();
    if (Math.abs(pixelWidth - previousPixelWidth) <= VisionConstants.kPixelWidthChangeThreshold) {
      pixelWidthStableCount++;
    } else {
      pixelWidthStableCount = 0;
    }
    previousPixelWidth = pixelWidth;

    return pixelWidthStableCount >= VisionConstants.kPixelWidthStableCounts;
  }

  private double getPixelWidthStable() {
    return isPixelWidthStable() ? 1.0 : 0.0;
  }

  private void resetVisionCheckSystem() {
    visionCheckTime.reset();
    visionCheckTime.start();
    numOfSerialChanges = 0;
  }

  public boolean isVisionWorking() {
    return !isVisionWorking;
  }

  @Override
  public void periodic() {
    fillBuffers();
    if (lastSerialNum != targetData.serial) {
      numOfSerialChanges++;
      lastSerialNum = targetData.serial;
    }
    if (visionCheckTime.hasElapsed(VisionConstants.kTimeForVisionCheck)) {
      if (numOfSerialChanges < VisionConstants.kNumOfVisionChecks) {
        logger.error("Deadeye is NOT working");
        isVisionWorking = false;
      } else {
        isVisionWorking = true;
      }
      resetVisionCheckSystem();
    }
  }
}
