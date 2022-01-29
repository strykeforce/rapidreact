// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
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

  public static final class DriveConstants {

    public static final double kDeadbandXLock = 0.2;
    public static final double kDeadbandAllStick = 0.05;

    // TODO: verify diameter and run calibration
    // 500 cm calibration = actual / odometry
    public static final double kWheelDiameterInches = 3.0 * (584.0 / 501.0);

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
  }

  public static final class MagazineConstants {
    public static final int kLowerMagazineTalonID = 06;
    public static final int kUpperMagazineTalonID = 12;
    public static final Color kBlueCargo =
        new Color(0.15, 0.35, 0.5); // FIXME need to get real color
    public static final Color kRedCargo =
        new Color(0.5, 0.25, 0.25); // FIXME need to get real color
    public static final Color kNoCargo = new Color(0.25, 0.5, 0.25); // FIXME need to get real color

    public static final double kMagazineIntakeSpeed = 0.5; // FIXME need an actual percentage

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
    public static final int kIntakeFalconID = 20;
    public static final double kIntakeSpeed = 0.5; // FIXME need an actual percentage

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
}
