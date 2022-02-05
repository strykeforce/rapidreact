// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;

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

  public Constants() {}

  public static final class DriveConstants {

    public static final double kDeadbandXLock = 0.2;
    public static final double kDeadbandAllStick = 0.10;

    // Contents for Sanity
    // Wheel math
    // Gear values
    //

    // TODO: verify diameter and run calibration
    // 500 cm calibration = actual / odometry
    public static final double kWheelDiameterInches = 3.0 * (554.0 / 500.0);

    // From: https://github.com/strykeforce/axis-config/
    public static final double kMaxSpeedMetersPerSecond = 3.889;

    public static final double kMaxOmega =
        (kMaxSpeedMetersPerSecond / Math.hypot(0.5461 / 2.0, 0.6477 / 2.0))
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
      final double x = 0.5461 / 2.0; // front-back, was ROBOT_LENGTH
      final double y = 0.6477 / 2.0; // left-right, was ROBOT_WIDTH
      Translation2d[] locs = new Translation2d[4];
      locs[0] = new Translation2d(x, y); // left front
      locs[1] = new Translation2d(x, -y); // right front
      locs[2] = new Translation2d(-x, y); // left rear
      locs[3] = new Translation2d(-x, -y); // right rear
      return locs;
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
      azimuthConfig.slot0.kF = 0.0;
      azimuthConfig.slot0.integralZone = 0;
      azimuthConfig.slot0.allowableClosedloopError = 0;
      azimuthConfig.slot0.maxIntegralAccumulator = 0;
      azimuthConfig.motionCruiseVelocity = 800;
      azimuthConfig.motionAcceleration = 10_000;
      azimuthConfig.velocityMeasurementWindow = 64;
      azimuthConfig.voltageCompSaturation = 12;
      return azimuthConfig;
    }

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

    public static final double kMinContourAreaSize = 100;
    public static final double kVerticalFov = 48.8;
    public static final double kHorizonFov = 1.012; // 50.8 //146 //radians 1.012 // deg 57.999
    public static final double kHorizonRes = 640; // 1280
    public static final double kTargetWidthIn = 39.5; // 34.6
    public static final double kCameraHeight = 20.75;
    public static final double kTargetHeight = 98.5;
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
    // + is left
    public static final double kHorizAngleCorrection = 2.5; // 2.5
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
    public static final double kWrapRange = 1;
    public static final double kTurretTicksPerDegree = 72.404; // 0.01745329 57.2957877856
    public static final double kTurretTicksPerRadian = 4148.444; // 4148.44421883
    public static final double kTurretMidpoint = 13_000;
    public static int kTurretZeroTicks = 1931;
    public static final int kForwardLimit = 26095; // 26000
    public static final int kReverseLimit = -100;
    public static final int kCloseEnoughTurret = 40;
    public static final double kMaxStringPotZero = 100;
    public static final double kMinStringPotZero = 0;
    public static final int kTurretId = 42;

    public static SupplyCurrentLimitConfiguration getSupplyCurrentLimitConfig() {
      return new SupplyCurrentLimitConfiguration(true, 5, 30, 500);
    }

    public static TalonSRXConfiguration getTurretTalonConfig() {
      TalonSRXConfiguration turretConfig = new TalonSRXConfiguration();
      turretConfig.forwardSoftLimitThreshold = Constants.TurretConstants.kForwardLimit;
      turretConfig.reverseSoftLimitThreshold = Constants.TurretConstants.kReverseLimit;
      turretConfig.forwardSoftLimitEnable = true;
      turretConfig.reverseSoftLimitEnable = true;
      turretConfig.slot0.kP = 2;
      turretConfig.slot0.kI = 0.01;
      turretConfig.slot0.kD = 80;
      turretConfig.slot0.kF = 0.21;
      turretConfig.slot0.integralZone = 40;
      turretConfig.slot0.maxIntegralAccumulator = 4500;
      turretConfig.voltageMeasurementFilter = 32;
      turretConfig.voltageCompSaturation = 12;
      turretConfig.motionCruiseVelocity = 4_000;
      turretConfig.motionAcceleration = 30_000;
      turretConfig.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled;
      turretConfig.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;
      return turretConfig;
    }
  }

  public static final class MagazineConstants {
    public static final int MagazineTalonID = 30;
    public static final Color kBlueCargo =
        new Color(0.15, 0.35, 0.5); // FIXME need to get real color
    public static final Color kRedCargo =
        new Color(0.5, 0.25, 0.25); // FIXME need to get real color
    public static final Color kNoCargo = new Color(0.25, 0.5, 0.25); // FIXME need to get real color

    public static TalonSRXConfiguration getMagazineTalonConfig() {
      TalonSRXConfiguration magazineConfig = new TalonSRXConfiguration();

      magazineConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
      magazineConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

      magazineConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      magazineConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

      magazineConfig.continuousCurrentLimit = 10;
      magazineConfig.peakCurrentDuration = 10;
      magazineConfig.peakCurrentLimit = 15;
      magazineConfig.slot0.kP = 0.0;
      magazineConfig.slot0.kI = 0.0;
      magazineConfig.slot0.kD = 0.0;
      magazineConfig.slot0.kF = 0.0;
      magazineConfig.slot0.integralZone = 0;
      magazineConfig.slot0.allowableClosedloopError = 0;
      magazineConfig.slot0.maxIntegralAccumulator = 0;
      magazineConfig.motionCruiseVelocity = 0;
      magazineConfig.motionAcceleration = 0;
      magazineConfig.velocityMeasurementWindow = 64;
      magazineConfig.voltageCompSaturation = 12;
      return magazineConfig;
    }
  }

  public static final class IntakeConstants {
    public static final int IntakeFalconID = 20;

    public static TalonFXConfiguration getIntakeFalconConfig() {
      TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
      intakeConfig.supplyCurrLimit.currentLimit = 10;
      intakeConfig.supplyCurrLimit.triggerThresholdCurrent = 15;
      intakeConfig.supplyCurrLimit.triggerThresholdTime = .02;
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

    public static final int kShooterFalconID = 40;
    public static final int kKickerFalconID = 41;
    public static final int kHoodTalonID = 42;

    public static int kHoodZeroTicks;
    public static final int kForwardSoftLimts = 1000;
    public static final int kReverseSoftLimits = 0;

    public static final int kStableCounts = 5;
    // FIX ME

    public static TalonFXConfiguration getShooterFalconConfig() {
      TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
      shooterConfig.supplyCurrLimit.currentLimit = 10;
      shooterConfig.supplyCurrLimit.triggerThresholdCurrent = 15;
      shooterConfig.supplyCurrLimit.triggerThresholdTime = .02;
      shooterConfig.supplyCurrLimit.enable = true;
      shooterConfig.slot0.kP = 0.0;
      shooterConfig.slot0.kI = 0.0;
      shooterConfig.slot0.kD = 0.0;
      shooterConfig.slot0.kF = 0.0;
      shooterConfig.slot0.integralZone = 0;
      shooterConfig.slot0.maxIntegralAccumulator = 0;
      shooterConfig.slot0.allowableClosedloopError = 0;
      shooterConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      shooterConfig.velocityMeasurementWindow = 64;
      shooterConfig.voltageCompSaturation = 12;
      return shooterConfig;
    }

    public static TalonFXConfiguration getKickerFalconConfig() {
      TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
      kickerConfig.supplyCurrLimit.currentLimit = 10;
      kickerConfig.supplyCurrLimit.triggerThresholdCurrent = 15;
      kickerConfig.supplyCurrLimit.triggerThresholdTime = .02;
      kickerConfig.supplyCurrLimit.enable = true;
      kickerConfig.slot0.kP = 0.0;
      kickerConfig.slot0.kI = 0.0;
      kickerConfig.slot0.kD = 0.0;
      kickerConfig.slot0.kF = 0.0;
      kickerConfig.slot0.integralZone = 0;
      kickerConfig.slot0.maxIntegralAccumulator = 0;
      kickerConfig.slot0.allowableClosedloopError = 0;
      kickerConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      kickerConfig.velocityMeasurementWindow = 64;
      kickerConfig.voltageCompSaturation = 12;
      return kickerConfig;
    }

    public static TalonSRXConfiguration getHoodTalonConfig() {
      TalonSRXConfiguration HoodConfig = new TalonSRXConfiguration();

      HoodConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
      HoodConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

      HoodConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      HoodConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

      HoodConfig.continuousCurrentLimit = 10;
      HoodConfig.peakCurrentDuration = 10;
      HoodConfig.peakCurrentLimit = 15;
      HoodConfig.slot0.kP = 0.0;
      HoodConfig.slot0.kI = 0.0;
      HoodConfig.slot0.kD = 0.0;
      HoodConfig.slot0.kF = 0.0;
      HoodConfig.slot0.integralZone = 0;
      HoodConfig.slot0.allowableClosedloopError = 0;
      HoodConfig.slot0.maxIntegralAccumulator = 0;
      HoodConfig.motionCruiseVelocity = 0;
      HoodConfig.motionAcceleration = 0;
      HoodConfig.velocityMeasurementWindow = 64;
      HoodConfig.voltageCompSaturation = 12;
      return HoodConfig;
    }
  }

  public static final class SmartDashboardConstants {
    public static final String kPitHoodOpenLoop = "Pit/Hood/hoodSpeed";
    public static final String kPitShooterOpenLoop = "Pit/Shooter/shooterSpeed";
    public static final String kTurretSetpointRadians = "Pit/Turret/SetpointRadians";
  }
}
