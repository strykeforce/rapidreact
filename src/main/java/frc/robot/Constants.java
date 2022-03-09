// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int kTalonConfigTimeout = 10; // ms
  public static final double kDeadeyePowerCycleTimeout = 5; // s

  public Constants() {}

  public static final class DriveConstants {
    // Drive Constants
    public static final double kWheelDiameterInches = 3.0 * (571.0 / 500.0); // Actual/Odometry

    // From: https://github.com/strykeforce/axis-config/
    public static final double kMaxSpeedMetersPerSecond = 3.889;
    public static final double kRobotWidth = 0.625;
    public static final double kRobotLength = 0.625;

    public static final double kMaxOmega =
        (kMaxSpeedMetersPerSecond / Math.hypot(kRobotWidth / 2.0, kRobotLength / 2.0))
            / 2.0; // wheel locations below

    // From: https://github.com/strykeforce/axis-config/
    static final double kDriveMotorOutputGear = 22;
    static final double kDriveInputGear = 48;
    static final double kBevelInputGear = 15;
    static final double kBevelOutputGear = 45;
    public static final double kDriveGearRatio =
        (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);

    static {
      // logger.debug("kMaxOmega = {}", kMaxOmega);
    }

    public static Translation2d[] getWheelLocationMeters() {
      final double x = kRobotLength / 2.0; // front-back, was ROBOT_LENGTH
      final double y = kRobotWidth / 2.0; // left-right, was ROBOT_WIDTH
      Translation2d[] locs = new Translation2d[4];
      locs[0] = new Translation2d(x, y); // left front
      locs[1] = new Translation2d(x, -y); // right front
      locs[2] = new Translation2d(-x, y); // left rear
      locs[3] = new Translation2d(-x, -y); // right rear
      return locs;
    }

    // Teleop Drive Constants
    public static final double kDeadbandXLock = 0.2;
    public static final double kDeadbandAllStick = 0.10;
    public static final double kCloseEnoughTicks = 10.0;

    // Climb Limits
    public static final double kMaxFwdStrStickClimb = 0.2 * kMaxSpeedMetersPerSecond;
    public static final double kMaxYawStickClimb = 0.2 * kMaxOmega;
    public static final double kYawJackFactorClimb = -0.4;

    // Default safety path constants
    public static final Pose2d startPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d endPose2d = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));

    public static ArrayList<Translation2d> getDefaultInternalWaypoints() {
      ArrayList<Translation2d> waypoints = new ArrayList<>();
      waypoints.add(new Translation2d(-0.5, 0));
      return waypoints;
    }

    public static TrajectoryConfig getDefaultTrajectoryConfig() {
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(1, 1);
      trajectoryConfig.setReversed(true);
      trajectoryConfig.setStartVelocity(0.0);
      trajectoryConfig.setEndVelocity(0.0);
      return trajectoryConfig;
    }

    // Azimuth Talon Config
    public static SupplyCurrentLimitConfiguration getAzimuthSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 10, 15, 0.04);
    }

    public static TalonSRXConfiguration getAzimuthTalonConfig() {
      // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
      TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();

      azimuthConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
      azimuthConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

      azimuthConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      azimuthConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

      azimuthConfig.continuousCurrentLimit = 10;
      azimuthConfig.peakCurrentDuration = 0;
      azimuthConfig.peakCurrentLimit = 0;
      azimuthConfig.slot0.kP = 10.0;
      azimuthConfig.slot0.kI = 0.0;
      azimuthConfig.slot0.kD = 100.0;
      azimuthConfig.slot0.kF = 1.0;
      azimuthConfig.slot0.integralZone = 0;
      azimuthConfig.slot0.allowableClosedloopError = 0;
      azimuthConfig.slot0.maxIntegralAccumulator = 0;
      azimuthConfig.motionCruiseVelocity = 800;
      azimuthConfig.motionAcceleration = 10_000;
      azimuthConfig.velocityMeasurementWindow = 64;
      azimuthConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      azimuthConfig.voltageCompSaturation = 12;
      return azimuthConfig;
    }

    // Drive Falcon Config
    public static TalonFXConfiguration getDriveTalonConfig() {
      TalonFXConfiguration driveConfig = new TalonFXConfiguration();
      driveConfig.supplyCurrLimit.currentLimit = 40;
      driveConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
      driveConfig.supplyCurrLimit.triggerThresholdTime = .04;
      driveConfig.supplyCurrLimit.enable = true;
      driveConfig.slot0.kP = 0.045;
      driveConfig.slot0.kI = 0.0005;
      driveConfig.slot0.kD = 0.000;
      driveConfig.slot0.kF = 0.047;
      driveConfig.slot0.integralZone = 500;
      driveConfig.slot0.maxIntegralAccumulator = 75_000;
      driveConfig.slot0.allowableClosedloopError = 0;
      driveConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      driveConfig.velocityMeasurementWindow = 64;
      driveConfig.voltageCompSaturation = 12;
      return driveConfig;
    }

    // Holonomic Controller Constants
    public static final double kPHolonomic = 6.0;
    public static final double kIHolonomic = 0.0;
    public static final double kDHolonomic = kPHolonomic / 100.0;

    public static final double kPOmega = 2.5;
    public static final double kIOmega = 0.0;
    public static final double kDOmega = 0.0;
    public static final double kMaxVelOmega = kMaxOmega / 2.0;
    public static final double kMaxAccelOmega = 3.14;
  }

  public static final class VisionConstants {
    // Actual 2022 Constants
    // + is left
    public static final double kHorizAngleCorrectionDegrees = 0.0; // 2.5 degrees

    // Old 2020 Constants
    public static final double kMinContourAreaSize = 100;
    public static final double kVerticalFov = 48.8;
    public static final double kHorizonFov = 1.012; // 50.8 //146 //radians 1.012 // deg 57.999
    public static final double kHorizonRes = 640; // 1280
    public static final double kTargetWidthIn = 5;
    public static final double kCameraHeight = 20.75;
    public static final double kSizeThreshold = 400;
    public static final double kDistanceThreshold = 200;
    public static final int kStableRange = 20;
    public static final int kStableCounts = 5;
    public static final double kCenteredRange = 2;
    public static final double kLostLimit = 30;
    public static final String kTablePath = "/home/lvuser/deploy/Lookup_Table.csv";
    public static final int kTableMin = 96;
    public static final int kTableMax = 360;
    public static final int kTableRes = 1;
    public static final int kShooterIndex = 2;
    public static final int kHoodIndex = 3;
    public static final double kTapeHeightIn = 101.625; // in
    public static final double kUpperHubRadiusIn = 26.6875;
    // + is further along track and lower
    public static final int kHoodInchesCorrectionR1 = 13; // 8-15 feet (was 20)
    public static final int kHoodInchesCorrectionR2 = 13; // 15-19 feet (old 10)
    public static final int kHoodInchesCorrectionR3 = 10; // 19-25 feet
    public static final int kHoodInchesCorrectionR4 = 20; // 25+ feet
    public static final int kHoodTicksPerInchR1 = 40; // 8-15 feet
    public static final int kHoodTicksPerInchR2 = 75; // 15-19 feet
    public static final int kHoodTicksPerInchR3 = 75; // 19-25 feet
    public static final int kHoodTicksPerInchR4 = 40; // 25+ feet
    public static String kCameraID = "A0";
  }

  public static final class TurretConstants {
    // Talon Constants
    public static final int kTurretId = 50;
    public static final double kFastCruiseVelocity = 4_000;
    public static final double kSlowCruiseVelocity = 2_000;
    public static final int kTurretZeroTicks = 756; // 1250
    public static final int kForwardLimit = 13_800; // 14
    public static final int kReverseLimit = -13_800; // 14
    public static final double kMaxStringPotZero = 100; // 2020 Robot
    public static final double kMinStringPotZero = 0; // 2020 Robot

    // Ticks -> Degrees/Radians
    public static final double kTurretTicksPerDegree =
        120.522; // 114.653     0.01745329 57.2957877856 72.404
    public static final double kTurretTicksPerRadian = 6905.414; // 6569.133
    public static final double kTurretMidpoint = 13_000;
    public static final double kWrapRange = 1;

    // Rotate Under Vision Constants
    public static final double kRotateByInitialKp = -0.4; // 0.4
    public static final double kRotateByFinalKp = -0.95; // 0.95
    public static final int kNotValidTargetCounts = 5; // how many frames to wait before seeking

    // Seek Constants
    public static final Translation2d kHubPositionMeters = new Translation2d(8.23, 4.11); // meters
    public static final Rotation2d kSeekAngleError = Rotation2d.fromDegrees(30); // 30 degrees
    public static final int kMaxSeekCount = 3; // 6
    public static final Rotation2d kTurretRobotOffset = Rotation2d.fromDegrees(180);

    // Close Enough & Stable Counts
    public static final int kCloseEnoughTicks = 40;
    public static final Rotation2d kCloseEnoughTarget = Rotation2d.fromDegrees(1); // 1
    public static final int kRotateByStableCounts = 3; // 3

    // Fender Shot Constants
    public static final Rotation2d kFenderAlliance = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d kFenderOpponent = Rotation2d.fromDegrees(90.0);

    // Talon Constants
    public static SupplyCurrentLimitConfiguration getSupplyCurrentLimitConfig() {
      return new SupplyCurrentLimitConfiguration(true, 10, 30, 0.5);
    }

    public static TalonSRXConfiguration getTurretTalonConfig() {
      TalonSRXConfiguration turretConfig = new TalonSRXConfiguration();

      turretConfig.forwardSoftLimitThreshold = Constants.TurretConstants.kForwardLimit;
      turretConfig.reverseSoftLimitThreshold = Constants.TurretConstants.kReverseLimit;
      turretConfig.forwardSoftLimitEnable = true;
      turretConfig.reverseSoftLimitEnable = true;
      turretConfig.slot0.kP = 2.5;
      turretConfig.slot0.kI = 0.04;
      turretConfig.slot0.kD = 100;
      turretConfig.slot0.kF = 0.13;
      turretConfig.slot0.integralZone = 100;
      turretConfig.slot0.maxIntegralAccumulator = 0;
      turretConfig.slot0.allowableClosedloopError = 15;
      turretConfig.voltageMeasurementFilter = 32;
      turretConfig.voltageCompSaturation = 12;
      turretConfig.motionCruiseVelocity = kFastCruiseVelocity; // 4_000
      turretConfig.motionAcceleration = 20_000;
      turretConfig.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled;
      turretConfig.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;
      turretConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      turretConfig.velocityMeasurementWindow = 64;

      return turretConfig;
    }
  }

  public static final class MagazineConstants {
    // Talon Constants
    public static final int kLowerMagazineTalonID = 30;
    public static final int kUpperMagazineTalonID = 31;

    // Color Sensor Constants
    public static final Color kBlueCargo =
        new Color(0.15, 0.35, 0.5); // FIXME need to get real color
    public static final Color kRedCargo =
        new Color(0.5, 0.25, 0.25); // FIXME need to get real color
    public static final Color kNoCargo = new Color(0.25, 0.5, 0.25); // FIXME need to get real color

    // Speed Constants
    public static final double kLowerMagazineIntakeSpeed = 8000; // 0.5
    public static final double kUpperMagazineIntakeSpeed = 2000;
    public static final double kUpperMagazineFeedSpeed = 2000; // 0.25
    public static final double kLowerMagazineIndexSpeed = 8000; // 0.35
    public static final double kUpperMagazineIndexSpeed = 2000;
    public static final double kMagazineEjectSpeed = -8000; // -0.5

    // State Machine Sequence Constants
    public static final double kShootDelay = 0.7;
    public static final double kEjectTimerDelay = 1.0;
    public static final double kShootUpperBeamStableCounts = 2;

    // Lower Magazine Talon Config
    public static SupplyCurrentLimitConfiguration getLowerMagazineCurrentLimit() {
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfig =
          new SupplyCurrentLimitConfiguration();

      supplyCurrentLimitConfig.currentLimit = 10.0;
      supplyCurrentLimitConfig.triggerThresholdCurrent = 20.0;
      supplyCurrentLimitConfig.triggerThresholdTime = 0.5;
      supplyCurrentLimitConfig.enable = true;

      return supplyCurrentLimitConfig;
    }

    public static TalonSRXConfiguration getLowerMagazineTalonConfig() {
      TalonSRXConfiguration magazineConfig = new TalonSRXConfiguration();

      magazineConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
      magazineConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

      magazineConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      magazineConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

      magazineConfig.slot0.kP = 0.1;
      magazineConfig.slot0.kI = 0.0;
      magazineConfig.slot0.kD = 10.0;
      magazineConfig.slot0.kF = 0.1;
      magazineConfig.slot0.integralZone = 0;
      magazineConfig.slot0.allowableClosedloopError = 0;
      magazineConfig.slot0.maxIntegralAccumulator = 0;
      magazineConfig.motionCruiseVelocity = 0;
      magazineConfig.motionAcceleration = 0;
      magazineConfig.velocityMeasurementWindow = 64;
      magazineConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      magazineConfig.voltageCompSaturation = 12;
      magazineConfig.voltageMeasurementFilter = 32;

      return magazineConfig;
    }

    // Upper Magazine Talon Config
    public static SupplyCurrentLimitConfiguration getUpperMagazineCurrentLimit() {
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfig =
          new SupplyCurrentLimitConfiguration();

      supplyCurrentLimitConfig.currentLimit = 10.0;
      supplyCurrentLimitConfig.triggerThresholdCurrent = 20.0;
      supplyCurrentLimitConfig.triggerThresholdTime = 0.5;
      supplyCurrentLimitConfig.enable = true;

      return supplyCurrentLimitConfig;
    }

    public static TalonSRXConfiguration getUpperMagazineTalonConfig() {
      TalonSRXConfiguration magazineConfig = new TalonSRXConfiguration();

      magazineConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
      magazineConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

      magazineConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      magazineConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

      magazineConfig.slot0.kP = 0.2;
      magazineConfig.slot0.kI = 0.0;
      magazineConfig.slot0.kD = 10.0;
      magazineConfig.slot0.kF = 0.21;
      magazineConfig.slot0.integralZone = 0;
      magazineConfig.slot0.allowableClosedloopError = 0;
      magazineConfig.slot0.maxIntegralAccumulator = 0;
      magazineConfig.motionCruiseVelocity = 0;
      magazineConfig.motionAcceleration = 0;
      magazineConfig.velocityMeasurementWindow = 64;
      magazineConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      magazineConfig.voltageCompSaturation = 12;
      magazineConfig.voltageMeasurementFilter = 32;

      return magazineConfig;
    }
  }

  public static final class ClimbConstants {
    // ID's
    public static final int kPivotArmFalconID = 60;
    public static final int kFixedArmFalconID = 61;
    public static final int kClimbShoulderId = 62;
    public static final int kPivotArmRatchetId = 0;
    public static final int kFixedArmRatchetId = 1;
    public static final int kLeftFixedHomeId = 0;
    public static final int kRightFixedHomeId = 1;

    // Home Sensor Constants
    public static final int kHomeSensorStableCounts = 2;

    // Manual Climb Constants
    public static final double kShoulderOffsetTicks = 10;
    public static final double kClimbArmsOpenLoopSpeed = 0.2;

    // Zeroing Constants
    public static final double kClimbArmsZeroSpeed = 0.1;
    public static final double kZeroTargetSpeedTicksPer100ms = 5;
    public static final double kZeroStableCounts = 25;
    public static final int kShoulderZeroTicks = 2675;
    public static final double kShoulderPostZeroTicks = 0;
    public static final double kPostZeroTickArmRatchetOn = -100;

    public static SupplyCurrentLimitConfiguration getZeroSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 5, 5, 0.1);
    }

    public static StatorCurrentLimitConfiguration getZeroStatorCurrentLimit() {
      return new StatorCurrentLimitConfiguration(true, 20, 20, 0.1);
    }

    // Servo Constants
    public static final double kPivotRatchetOn = 0.5;
    public static final double kFixedRatchetOn = 0.5;
    public static final double kPivotRatchetOff = 0.0;
    public static final double kFixedRatchetOff = 0.0;
    public static final double kDisengageRatchetSpeed = 0.2;
    public static final double kDisengageRatchetTicks = 100;
    public static final double kDisengageRatchetServoTimer = 0.2;

    // Closed Loop Movement Constants
    public static final double kFixedArmCloseEnough = 1_000.0;
    public static final double kPivotArmCloseEnough = 1_000.0;
    public static final double kShoulderCloseEnough = 100.0;
    public static final int kFixedArmStableCounts = 2;
    public static final int kPivotArmStableCounts = 2;
    public static final int kShoulderStableCounts = 2;
    public static final double kShoulderCriuiseVelDefault = 1_000;

    // Open Loop Movement Constants
    public static final double kFixedArmExtendSpeed = -0.25;
    public static final double kFixedArmRetractSpeed = 0.25;
    public static final double kPivotArmExtendSpeed = -0.25;
    public static final double kPivotArmRetractSpeed = 0.25;

    // Climb States -> Desired Endpoint in Ticks
    public static final double kFMidExtTicks = -210_000;
    public static final double kFMidRetSt1Ticks = -135_000;
    public static final double kFMidRetSt2Ticks = -115_000;
    public static final double kFMidRetSt3Ticks = -5_000;
    public static final double kHighPvtBck1Ticks = 5_500;
    public static final double kPHighExtTicks = -197_000;
    public static final double kHighPvtFwd1Ticks = 3_200;
    public static final double kPHighRet1Ticks = -168_000;
    public static final double kHighPvtBck2Ticks = 4_500;
    public static final double kFHighExtSt1Ticks = -80_000;
    public static final double kFHighExtSt2Ticks = -120_000;
    public static final double kFHighExtSt3Ticks = -140_000;
    public static final double kHighPvtBck3Ticks = -3_000;
    public static final double kPHighRet2Ticks = -120_000;
    public static final double kHighPvtFwd2Ticks = 750;
    public static final double kFTvsRet1Ticks = -1_000;
    public static final double kTvsPvtBck1Ticks = 5_000;
    public static final double kPTvsExtTicks = -200_000;
    public static final double kTvsPvtFwdTicks = 3_300;
    public static final double kPTvsRetTicks = -168_000;
    public static final double kTvsPvtBck2Ticks = 4_500;
    public static final double kFTvsExtTicks = -140_000;
    public static final double kTvsPvtBck3Ticks = 2_700;
    public static final double kFTvsRet2Ticks = -5_000;
    public static final double kFHighExtFinTicks = -190_000;
    public static final double kHighPvtBckFinTicks = 700;
    public static final double kFHighRetFinTicks = -175_000;

    // Climb States -> Desired Open Loop Speed (Arms only, Shoulder = closed loop)
    public static final double kFMidExtSpeed = -0.8;
    public static final double kFMidRetSt1Speed = 0.5;
    public static final double kFMidRetSt2Speed = 0.16;
    public static final double kFMidRetSt3Speed = 0.5;
    public static final double kPHighExtSpeed = -0.5;
    public static final double kPHighRet1Speed = 0.12;
    public static final double kFHighExtSt1Speed = -0.35;
    public static final double kFHighExtSt2Speed = -0.1;
    public static final double kFHighExtSt3Speed = -0.4;
    public static final double kPHighRet2Speed = 0.3;
    public static final double kFTvsRet1Speed = 0.25;
    public static final double kPTvsExtSpeed = -0.25;
    public static final double kPTvsRetSpeed = 0.25;
    public static final double kFTvsExtSpeed = -0.25;
    public static final double kFTvsRet2Speed = 0.25;
    public static final double kFHighExtFinSpeed = -0.5;
    public static final double kFHighRetFinSpeed = 0.2;

    // Climb States -> Desired Shoulder Cruise Vel
    public static final double kHighPvtBk1Vel = 1_000;
    public static final double kHighPvtFwd1Vel = 1_000;
    public static final double kHighPvtBk2Vel = 1_000;
    public static final double kHighPvtBk3Vel = 500;
    public static final double kHighPvtFwd2Vel = 500;
    public static final double kTvsPvtBk1Vel = 1_000;
    public static final double kTvsPvtFwdVel = 1_000;
    public static final double kTvsPvtBk2Vel = 1_000;
    public static final double kTvsPvtBk3Vel = 1_000;
    public static final double kHighPvtBkFinVel = 100;

    // Pivot Arm Falcon Config
    public static SupplyCurrentLimitConfiguration getPivotArmSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 80, 90, 0.1);
    }

    public static StatorCurrentLimitConfiguration getPivotArmStatorCurrentLimit() {
      return new StatorCurrentLimitConfiguration(true, 100, 120, 0.1);
    }

    public static TalonFXConfiguration getPivotArmFalconConfig() {
      TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
      pivotConfig.supplyCurrLimit.currentLimit = 80;
      pivotConfig.supplyCurrLimit.triggerThresholdCurrent = 90;
      pivotConfig.supplyCurrLimit.triggerThresholdTime = 0.1;
      pivotConfig.supplyCurrLimit.enable = true;
      pivotConfig.statorCurrLimit.currentLimit = 100.0;
      pivotConfig.statorCurrLimit.triggerThresholdCurrent = 120.0;
      pivotConfig.statorCurrLimit.triggerThresholdTime = 0.1;
      pivotConfig.statorCurrLimit.enable = true;
      pivotConfig.slot0.kP = 1.0;
      pivotConfig.slot0.kI = 0.0;
      pivotConfig.slot0.kD = 0.0;
      pivotConfig.slot0.kF = 0.065;
      pivotConfig.slot0.integralZone = 0;
      pivotConfig.slot0.maxIntegralAccumulator = 0;
      pivotConfig.slot0.allowableClosedloopError = 0;
      pivotConfig.motionCruiseVelocity = 5_000;
      pivotConfig.motionAcceleration = 30_000;
      pivotConfig.forwardSoftLimitEnable = false;
      pivotConfig.forwardSoftLimitThreshold = 0;
      pivotConfig.reverseSoftLimitEnable = false;
      pivotConfig.reverseSoftLimitThreshold = -200_000;
      pivotConfig.neutralDeadband = 0.01;
      pivotConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      pivotConfig.velocityMeasurementWindow = 64;
      pivotConfig.voltageCompSaturation = 12;
      pivotConfig.voltageMeasurementFilter = 32;
      return pivotConfig;
    }

    // Fixed Arm Falcon Config
    public static SupplyCurrentLimitConfiguration getFixedArmSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 80, 90, 0.1);
    }

    public static StatorCurrentLimitConfiguration getFixedArmStatorCurrentLimit() {
      return new StatorCurrentLimitConfiguration(true, 100, 120, 0.1);
    }

    public static TalonFXConfiguration getFixedArmFalconConfig() {
      TalonFXConfiguration fixedConfig = new TalonFXConfiguration();
      fixedConfig.supplyCurrLimit.currentLimit = 80;
      fixedConfig.supplyCurrLimit.triggerThresholdCurrent = 90;
      fixedConfig.supplyCurrLimit.triggerThresholdTime = 0.1;
      fixedConfig.supplyCurrLimit.enable = true;
      fixedConfig.statorCurrLimit.currentLimit = 100.0;
      fixedConfig.statorCurrLimit.triggerThresholdCurrent = 120.0;
      fixedConfig.statorCurrLimit.triggerThresholdTime = 0.1;
      fixedConfig.statorCurrLimit.enable = true;
      fixedConfig.slot0.kP = 1.0;
      fixedConfig.slot0.kI = 0.0;
      fixedConfig.slot0.kD = 0.0;
      fixedConfig.slot0.kF = 0.065;
      fixedConfig.slot0.integralZone = 0;
      fixedConfig.slot0.maxIntegralAccumulator = 0;
      fixedConfig.slot0.allowableClosedloopError = 0;
      fixedConfig.motionCruiseVelocity = 5_000;
      fixedConfig.motionAcceleration = 30_000;
      fixedConfig.forwardSoftLimitEnable = false;
      fixedConfig.forwardSoftLimitThreshold = 0;
      fixedConfig.reverseSoftLimitEnable = false;
      fixedConfig.reverseSoftLimitThreshold = -220_000;
      fixedConfig.neutralDeadband = 0.01;
      fixedConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      fixedConfig.velocityMeasurementWindow = 64;
      fixedConfig.voltageCompSaturation = 12;
      fixedConfig.voltageMeasurementFilter = 32;
      return fixedConfig;
    }

    // Shoulder Talon Config
    public static SupplyCurrentLimitConfiguration getShoulderCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 3, 3, 0.1);
    }

    public static TalonSRXConfiguration getShoulderTalonConfig() {
      TalonSRXConfiguration ShoulderConfig = new TalonSRXConfiguration();

      ShoulderConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
      ShoulderConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

      ShoulderConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      ShoulderConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

      ShoulderConfig.slot0.kP = 3.0;
      ShoulderConfig.slot0.kI = 0.0;
      ShoulderConfig.slot0.kD = 15.0;
      ShoulderConfig.slot0.kF = 0.55;
      ShoulderConfig.slot0.integralZone = 0;
      ShoulderConfig.slot0.allowableClosedloopError = 0;
      ShoulderConfig.slot0.maxIntegralAccumulator = 0;
      ShoulderConfig.motionCruiseVelocity = 1_000;
      ShoulderConfig.motionAcceleration = 5_000;
      ShoulderConfig.velocityMeasurementWindow = 64;
      ShoulderConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      ShoulderConfig.voltageCompSaturation = 12;
      ShoulderConfig.voltageMeasurementFilter = 32;
      ShoulderConfig.forwardSoftLimitEnable = true;
      ShoulderConfig.forwardSoftLimitThreshold = 9500;
      ShoulderConfig.reverseSoftLimitEnable = true;
      ShoulderConfig.reverseSoftLimitThreshold = -6000;

      return ShoulderConfig;
    }
  }

  public static final class IntakeConstants {
    public static final int kIntakeFalconID = 20;
    public static final double kIntakeSpeed = 0.5;
    public static final double kIntakeEjectSpeed = -0.5;

    public static TalonFXConfiguration getIntakeFalconConfig() {
      TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
      intakeConfig.supplyCurrLimit.currentLimit = 10;
      intakeConfig.supplyCurrLimit.triggerThresholdCurrent = 15;
      intakeConfig.supplyCurrLimit.triggerThresholdTime = 0.5;
      intakeConfig.supplyCurrLimit.enable = true;
      intakeConfig.slot0.kP = 0.0;
      intakeConfig.slot0.kI = 0.0;
      intakeConfig.slot0.kD = 0.0;
      intakeConfig.slot0.kF = 0.0;
      intakeConfig.slot0.integralZone = 0;
      intakeConfig.slot0.maxIntegralAccumulator = 0;
      intakeConfig.slot0.allowableClosedloopError = 0;
      intakeConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      intakeConfig.velocityMeasurementWindow = 64;
      intakeConfig.voltageCompSaturation = 12;
      return intakeConfig;
    }
  }

  public static final class ShooterConstants {
    // ID's and Locations
    public static final int kShooterFalconID = 40;
    public static final int kKickerFalconID = 41;
    public static final int kHoodTalonID = 42;
    public static final String kLookupTablePath = "/home/lvuser/deploy/LookupTable.csv";

    // Lookup Table Consta
    public static final double kLookupMinPixel = 160;
    public static final double kLookupMaxPixel = 380;
    public static final double kNumRows = 221;
    public static final double kLookupRes = 1.0;

    // Hood Encoder Constants
    public static final int kHoodZeroTicks = 1800;
    public static final int kForwardSoftLimts = 5800;
    public static final int kReverseSoftLimits = 100;

    // Arm Shooter Constants
    public static final double kKickerArmTicksP100ms = 4000;
    public static final double kShooterArmTicksP100ms = 4000;

    // Opponent Cargo Constants
    public static final double kKickerOpTicksP100ms = 4000;
    public static final double kShooterOpTicksP100ms = 4000;
    public static final double kHoodOpTicks = 5800;

    // High Fender Shot Constants
    public static final double kKickerFenderHighTicksP100ms = 2000;
    public static final double kShooterFenderHighTicksP100ms = 10000;
    public static final double kHoodFenderHighTicks = 0;

    // Low Fender Shot Constants
    public static final double kKickerFenderLowTicksP100ms = 0; // 3000
    public static final double kShooterFenderLowTicksP100ms = 6500; // 4000
    public static final double kHoodFenderLowTicks = 0;

    public static final int kStableCounts = 5; // FIX ME
    public static final double kCloseEnoughTicksP100ms = 100;
    public static final double kCloseEnoughTicks = 100;

    public static TalonFXConfiguration getShooterFalconConfig() {
      TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
      shooterConfig.supplyCurrLimit.currentLimit = 40;
      shooterConfig.supplyCurrLimit.triggerThresholdCurrent = 1;
      shooterConfig.supplyCurrLimit.triggerThresholdTime = 0.001;
      shooterConfig.supplyCurrLimit.enable = true;
      shooterConfig.statorCurrLimit.currentLimit = 80.0;
      shooterConfig.statorCurrLimit.triggerThresholdCurrent = 1.0;
      shooterConfig.statorCurrLimit.triggerThresholdTime = 0.001;
      shooterConfig.statorCurrLimit.enable = true;
      shooterConfig.slot0.kP = 0.2;
      shooterConfig.slot0.kI = 0.0035;
      shooterConfig.slot0.kD = 2.0;
      shooterConfig.slot0.kF = 0.0465;
      shooterConfig.slot0.integralZone = 200;
      shooterConfig.slot0.maxIntegralAccumulator = 20_000;
      shooterConfig.slot0.allowableClosedloopError = 0;
      shooterConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_20Ms;
      shooterConfig.velocityMeasurementWindow = 16;
      shooterConfig.voltageCompSaturation = 0;
      shooterConfig.voltageMeasurementFilter = 32;
      shooterConfig.peakOutputForward = 1.0;
      shooterConfig.peakOutputReverse = -0.1;

      return shooterConfig;
    }

    public static TalonFXConfiguration getKickerFalconConfig() {
      TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
      kickerConfig.supplyCurrLimit.currentLimit = 40;
      kickerConfig.supplyCurrLimit.triggerThresholdCurrent = 1;
      kickerConfig.supplyCurrLimit.triggerThresholdTime = 0.001;
      kickerConfig.supplyCurrLimit.enable = true;
      kickerConfig.statorCurrLimit.currentLimit = 80.0;
      kickerConfig.statorCurrLimit.triggerThresholdCurrent = 1.0;
      kickerConfig.statorCurrLimit.triggerThresholdTime = 0.001;
      kickerConfig.statorCurrLimit.enable = true;
      kickerConfig.slot0.kP = 0.2;
      kickerConfig.slot0.kI = 0.0035;
      kickerConfig.slot0.kD = 0.5;
      kickerConfig.slot0.kF = 0.0472;
      kickerConfig.slot0.integralZone = 200;
      kickerConfig.slot0.maxIntegralAccumulator = 20_000;
      kickerConfig.slot0.allowableClosedloopError = 0;
      kickerConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_20Ms;
      kickerConfig.velocityMeasurementWindow = 16;
      kickerConfig.voltageCompSaturation = 0;
      kickerConfig.voltageMeasurementFilter = 32;
      kickerConfig.peakOutputForward = 1.0;
      kickerConfig.peakOutputReverse = -0.1;

      return kickerConfig;
    }

    public static SupplyCurrentLimitConfiguration getHoodCurrentLimit() {
      SupplyCurrentLimitConfiguration hoodCurrentLimit = new SupplyCurrentLimitConfiguration();

      hoodCurrentLimit.currentLimit = 10.0;
      hoodCurrentLimit.triggerThresholdCurrent = 15.0;
      hoodCurrentLimit.triggerThresholdTime = 0.04;
      hoodCurrentLimit.enable = true;

      return hoodCurrentLimit;
    }

    public static TalonSRXConfiguration getHoodTalonConfig() {
      TalonSRXConfiguration hoodConfig = new TalonSRXConfiguration();

      hoodConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
      hoodConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

      hoodConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      hoodConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

      hoodConfig.slot0.kP = 6.0;
      hoodConfig.slot0.kI = 0.0;
      hoodConfig.slot0.kD = 120.0;
      hoodConfig.slot0.kF = 0.57;
      hoodConfig.slot0.integralZone = 0;
      hoodConfig.slot0.maxIntegralAccumulator = 0;
      hoodConfig.slot0.allowableClosedloopError = 0;
      hoodConfig.motionCruiseVelocity = 1500;
      hoodConfig.motionAcceleration = 80_000;
      hoodConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      hoodConfig.velocityMeasurementWindow = 64;
      hoodConfig.voltageCompSaturation = 12;
      hoodConfig.voltageMeasurementFilter = 32;

      return hoodConfig;
    }
  }

  public static final class DashboardConstants {
    public static final double kLeftStickDeadBand = 0.1;
    public static final double kRightStickDeadBand = 0.1;
    public static final double kTriggerDeadBand = 0.1;
    public static final String kPitHoodSetpointTicks = "Tune/Hood/hoodPos";
    public static final String kPitShooterSetpointTicks = "Tune/Shooter/shooterSpeed";
    public static final String kTurretSetpointRadians = "Pit/Turret/SetpointRadians";
    public static final String kPitKickerSetpointTicks = "Tune/Kicker/kickerSpeed";
    public static final String kTuneUpperMagSpeedTicks = "Tune/Magazine/UpperSpeed";
    public static final int kLockoutBNCid = 8;
  }

  public static final class AutoConstants {
    public static final int kStartSwitchId = 2;
    public static final int kEndSwitchId = 7;
    public static final int kStableCounts = 100; // s
    public static final Rotation2d kLeftStartYaw = Rotation2d.fromDegrees(136.5);
    public static final Rotation2d kMidStartYaw = Rotation2d.fromDegrees(-156.0);
    public static final Rotation2d kRightStartYaw = Rotation2d.fromDegrees(-88.5);
  }
}
