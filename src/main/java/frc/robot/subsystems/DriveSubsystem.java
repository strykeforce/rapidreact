package frc.robot.subsystems;

import static frc.robot.Constants.kTalonConfigTimeout;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants.DriveConstants;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Set;
import net.consensys.cava.toml.Toml;
import net.consensys.cava.toml.TomlArray;
import net.consensys.cava.toml.TomlParseResult;
import net.consensys.cava.toml.TomlTable;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.TalonSwerveModule;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DriveSubsystem extends MeasurableSubsystem {

  private static final Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
  private final SwerveDrive swerveDrive;
  private final HolonomicDriveController holonomicController;
  private final ProfiledPIDController omegaController;
  private final PIDController xController;
  private final PIDController yController;
  private double[] desiredAzimuthPositions = new double[4];
  // Grapher Variables
  private ChassisSpeeds holoContOutput = new ChassisSpeeds();
  private State holoContInput = new State();
  private Rotation2d holoContAngle = new Rotation2d();
  private Double trajectoryActive = 0.0;

  public DriveSubsystem(boolean isThrifty) {

    if (!isThrifty) {
      var moduleBuilder =
          new TalonSwerveModule.Builder()
              .driveGearRatio(DriveConstants.kDriveGearRatio)
              .wheelDiameterInches(DriveConstants.kWheelDiameterInches)
              .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond);

      TalonSwerveModule[] swerveModules = new TalonSwerveModule[4];
      Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();

      for (int i = 0; i < 4; i++) {
        var azimuthTalon = new TalonSRX(i);
        azimuthTalon.configFactoryDefault(kTalonConfigTimeout);
        azimuthTalon.configAllSettings(DriveConstants.getAzimuthTalonConfig(), kTalonConfigTimeout);
        azimuthTalon.enableCurrentLimit(true);
        azimuthTalon.enableVoltageCompensation(true);
        azimuthTalon.setNeutralMode(NeutralMode.Coast);

        var driveTalon = new TalonFX(i + 10);
        driveTalon.configFactoryDefault(kTalonConfigTimeout);
        driveTalon.configAllSettings(DriveConstants.getDriveTalonConfig(), kTalonConfigTimeout);
        driveTalon.enableVoltageCompensation(true);
        driveTalon.setNeutralMode(NeutralMode.Brake);

        swerveModules[i] =
            moduleBuilder
                .azimuthTalon(azimuthTalon)
                .driveTalon(driveTalon)
                .wheelLocationMeters(wheelLocations[i])
                .build();

        swerveModules[i].loadAndSetAzimuthZeroReference();
      }

      swerveDrive = new SwerveDrive(swerveModules);
      swerveDrive.resetGyro();
      swerveDrive.setGyroOffset(Rotation2d.fromDegrees(180));
    } else {
      final double kDriveMotorOutputGear = 21;
      final double kDriveInputGear = 12;
      final double kBevelInputGear = 45;
      final double kBevelOutputGear = 15;
      final double kDriveGearRatio =
          (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);

      double robotLength = 0.6;
      double robotWidth = 0.6;

      var moduleBuilder =
          new TalonSwerveModule.Builder()
              .driveGearRatio(kDriveGearRatio)
              .wheelDiameterInches(3.0)
              .driveMaximumMetersPerSecond(3.889);

      TalonSwerveModule[] swerveModules = new TalonSwerveModule[4];
      Translation2d[] wheelLocations = new Translation2d[4];
      wheelLocations[0] = new Translation2d(robotLength / 2.0, robotWidth / 2.0);
      wheelLocations[1] = new Translation2d(robotLength / 2.0, -robotWidth / 2.0);
      wheelLocations[2] = new Translation2d(-robotLength / 2.0, robotWidth / 2.0);
      wheelLocations[3] = new Translation2d(-robotLength / 2.0, -robotWidth / 2.0);

      TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();
      azimuthConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
      azimuthConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      azimuthConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;
      azimuthConfig.continuousCurrentLimit = 10;
      azimuthConfig.peakCurrentLimit = 0;
      azimuthConfig.peakCurrentDuration = 0;
      azimuthConfig.slot0.kP = 10.0;
      azimuthConfig.slot0.kI = 0.0;
      azimuthConfig.slot0.kD = 100.0;
      azimuthConfig.slot0.kF = 0.0;
      azimuthConfig.slot0.integralZone = 0;
      azimuthConfig.slot0.allowableClosedloopError = 0;
      azimuthConfig.motionCruiseVelocity = 800;
      azimuthConfig.motionAcceleration = 10_000;
      azimuthConfig.velocityMeasurementWindow = 64;
      azimuthConfig.voltageCompSaturation = 12;

      TalonSRXConfiguration driveConfig = new TalonSRXConfiguration();
      driveConfig.continuousCurrentLimit = 40;
      driveConfig.peakCurrentLimit = 45;
      driveConfig.peakCurrentDuration = 40;
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

      for (int i = 0; i < 4; i++) {
        var azimuthTalon = new TalonSRX(i);
        azimuthTalon.configFactoryDefault(kTalonConfigTimeout);
        azimuthTalon.configAllSettings(azimuthConfig, kTalonConfigTimeout);
        azimuthTalon.enableCurrentLimit(true);
        azimuthTalon.enableVoltageCompensation(true);
        azimuthTalon.setNeutralMode(NeutralMode.Coast);
        azimuthTalon.setInverted(true);

        var driveTalon = new TalonSRX(i + 10);
        driveTalon.configFactoryDefault(kTalonConfigTimeout);
        driveTalon.configAllSettings(driveConfig, kTalonConfigTimeout);
        driveTalon.enableCurrentLimit(true);
        driveTalon.enableVoltageCompensation(true);
        driveTalon.setNeutralMode(NeutralMode.Brake);

        swerveModules[i] =
            moduleBuilder
                .azimuthTalon(azimuthTalon)
                .driveTalon(driveTalon)
                .wheelLocationMeters(wheelLocations[i])
                .build();

        loadAndSetAzimuthZeroReference(swerveModules[i], i);
      }
      swerveDrive = new SwerveDrive(swerveModules);
      swerveDrive.resetGyro();
      swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0.0));
    }

    // Setup Holonomic Controller
    omegaController =
        new ProfiledPIDController(
            DriveConstants.kPOmega,
            DriveConstants.kIOmega,
            DriveConstants.kDOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxOmega, DriveConstants.kMaxAccelOmega));
    omegaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
    xController =
        new PIDController(
            DriveConstants.kPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
    yController =
        new PIDController(
            DriveConstants.kPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
    holonomicController = new HolonomicDriveController(xController, yController, omegaController);
    // Disabling the holonomic controller makes the robot directly follow the trajectory output (no
    // closing the loop on x,y,theta errors)
    holonomicController.setEnabled(true);
  }

  private void loadAndSetAzimuthZeroReference(TalonSwerveModule module, int i) {
    int index = i;
    String key = String.format("SwerveDrive/wheel.%d", index);
    int reference = Preferences.getInt(key, Integer.MIN_VALUE);
    if (reference == Integer.MIN_VALUE) {
      logger.error("no saved azimuth zero reference for swerve module {}", index);
    }
    logger.info("swerve module {}: loaded azimuth zero reference = {}", index, reference);

    int azimuthAbsoluteCounts =
        module.getAzimuthTalon().getSensorCollection().getPulseWidthPosition() & 0xFFF;
    logger.info("swerve module {}: azimuth absolute position = {}", index, azimuthAbsoluteCounts);

    int azimuthSetpoint = reference - azimuthAbsoluteCounts;
    ErrorCode errorCode =
        module.getAzimuthTalon().setSelectedSensorPosition(azimuthSetpoint, 0, 10);
    if (errorCode.value != 0) {
      logger.error("Talon error code while setting azimuth zero: {}", errorCode);
    }

    module.getAzimuthTalon().set(ControlMode.MotionMagic, azimuthSetpoint);
    logger.info("swerve module {}: set azimuth encoder = {}", index, azimuthSetpoint);
  }

  public void drive(
      double forwardMetersPerSec, double strafeMetersPerSec, double yawRadiansPerSec) {
    swerveDrive.drive(forwardMetersPerSec, strafeMetersPerSec, yawRadiansPerSec, true);
  }

  // Closed-Loop (Velocity-Controlled) Swerve Movements
  public void move(
      double forwardMetersPerSec,
      double strafeMetersPerSec,
      double yawRadiansPerSec,
      Boolean isFieldOriented) {
    swerveDrive.move(forwardMetersPerSec, strafeMetersPerSec, yawRadiansPerSec, isFieldOriented);
  }

  @Override
  public void periodic() {
    swerveDrive.periodic();
  }

  public void resetGyro() {
    swerveDrive.resetGyro();
  }

  public void setGyroOffset(Rotation2d rotation) {
    swerveDrive.setGyroOffset(rotation);
  }

  public void lockZero() {
    SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(0.0));
    }

    logger.info("Locking wheels to zero");
  }

  public void xLock() {
    SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
    for (int i = 0; i < 4; i++) {
      if (i == 1 || i == 2) {
        swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(-45.0));
      }
      if (i == 0 || i == 3) {
        swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(45.0));
      }
    }

    logger.info("Locking wheels to X");
  }

  private SwerveModuleState[] getSwerveModuleStates() {
    TalonSwerveModule[] swerveModules = (TalonSwerveModule[]) swerveDrive.getSwerveModules();
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      swerveModuleStates[i] = swerveModules[i].getState();
    }

    return swerveModuleStates;
  }

  public Rotation2d getGyroRotation2d() {
    return swerveDrive.getHeading();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
    xController.reset();
    yController.reset();
    omegaController.reset(pose.getRotation().getRadians());
    logger.info("reset odometry with: {}", pose);
  }

  public Pose2d getPoseMeters() {
    return swerveDrive.getPoseMeters();
  }

  // Trajectory TOML Parsing
  public PathData generateTrajectory(String trajectoryName) {

    try {
      TomlParseResult parseResult =
          Toml.parse(Paths.get("/home/lvuser/deploy/paths/" + trajectoryName + ".toml"));
      logger.info("Generating Trajectory: {}", trajectoryName);
      Pose2d startPose = parsePose2d(parseResult, "start_pose");
      Pose2d endPose = parsePose2d(parseResult, "end_pose");
      TomlArray internalPointsToml = parseResult.getArray("internal_points");
      ArrayList<Translation2d> path = new ArrayList<>();
      logger.info("Toml Internal Points Array Size: {}", internalPointsToml.size());

      for (int i = 0; i < internalPointsToml.size(); i++) {
        TomlTable waypointToml = internalPointsToml.getTable(i);
        Translation2d waypoint =
            new Translation2d(waypointToml.getDouble("x"), waypointToml.getDouble("y"));
        path.add(waypoint);
      }

      // Trajectory Config parsed from toml - any additional constraints would be added here
      TrajectoryConfig trajectoryConfig =
          new TrajectoryConfig(
              parseResult.getDouble("max_velocity"), parseResult.getDouble("max_acceleration"));
      trajectoryConfig.setReversed(parseResult.getBoolean("is_reversed"));
      trajectoryConfig.setStartVelocity(parseResult.getDouble("start_velocity"));
      trajectoryConfig.setEndVelocity(parseResult.getDouble("end_velocity"));

      double yawDegrees = parseResult.getDouble("target_yaw");
      Rotation2d targetYaw = Rotation2d.fromDegrees(yawDegrees);
      logger.info("Yaw is {}", targetYaw);

      Trajectory trajectoryGenerated =
          TrajectoryGenerator.generateTrajectory(startPose, path, endPose, trajectoryConfig);
      return new PathData(targetYaw, trajectoryGenerated);
    } catch (Exception error) {
      logger.error(error.toString());
      logger.error("Path {} not found", trajectoryName);
      throw new RuntimeException(error);
    }
  }

  private Pose2d parsePose2d(TomlParseResult parseResult, String pose) {
    return new Pose2d(
        parseResult.getTable(pose).getDouble("x"),
        parseResult.getTable(pose).getDouble("y"),
        Rotation2d.fromDegrees(parseResult.getTable(pose).getDouble("angle")));
  }

  // Holonomic Controller
  public void calculateController(State desiredState, Rotation2d desiredAngle) {
    holoContInput = desiredState;
    holoContAngle = desiredAngle;
    holoContOutput = holonomicController.calculate(getPoseMeters(), desiredState, desiredAngle);
    move(
        holoContOutput.vxMetersPerSecond,
        holoContOutput.vyMetersPerSecond,
        holoContOutput.omegaRadiansPerSecond,
        false);
  }

  public void setEnableHolo(boolean enabled) {
    holonomicController.setEnabled(enabled);
    logger.info("Holonomic Controller Enabled: {}", enabled);
  }

  // Make whether a trajectory is currently active obvious on grapher
  public void grapherTrajectoryActive(Boolean active) {
    if (active) trajectoryActive = 1.0;
    else trajectoryActive = 0.0;
  }

  @Override
  public void registerWith(@NotNull TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    swerveDrive.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Gyro Rotation2D(deg)", () -> swerveDrive.getHeading().getDegrees()),
        new Measure("Odometry X", () -> swerveDrive.getPoseMeters().getX()),
        new Measure("Odometry Y", () -> swerveDrive.getPoseMeters().getY()),
        new Measure(
            "Odometry Rotation2D(deg)",
            () -> swerveDrive.getPoseMeters().getRotation().getDegrees()),
        new Measure("Trajectory Vel", () -> holoContInput.velocityMetersPerSecond),
        new Measure("Trajectory Accel", () -> holoContInput.accelerationMetersPerSecondSq),
        new Measure("Trajectory X", () -> holoContInput.poseMeters.getX()),
        new Measure("Trajectory Y", () -> holoContInput.poseMeters.getY()),
        new Measure(
            "Trajectory Rotation2D(deg)",
            () -> holoContInput.poseMeters.getRotation().getDegrees()),
        new Measure("Desired Gyro Heading(deg)", () -> holoContAngle.getDegrees()),
        new Measure("Holonomic Cont Vx", () -> holoContOutput.vxMetersPerSecond),
        new Measure("Holonomic Cont Vy", () -> holoContOutput.vyMetersPerSecond),
        new Measure("Holonomic Cont Vomega", () -> holoContOutput.omegaRadiansPerSecond),
        new Measure("Trajectory Active", () -> trajectoryActive),
        new Measure("Wheel 0 Angle", () -> getSwerveModuleStates()[0].angle.getDegrees()),
        new Measure("Wheel 0 Speed", () -> getSwerveModuleStates()[0].speedMetersPerSecond),
        new Measure("Wheel 1 Angle", () -> getSwerveModuleStates()[1].angle.getDegrees()),
        new Measure("Wheel 1 Speed", () -> getSwerveModuleStates()[1].speedMetersPerSecond),
        new Measure("Wheel 2 Angle", () -> getSwerveModuleStates()[2].angle.getDegrees()),
        new Measure("Wheel 2 Speed", () -> getSwerveModuleStates()[2].speedMetersPerSecond),
        new Measure("Wheel 3 Angle", () -> getSwerveModuleStates()[3].angle.getDegrees()),
        new Measure("Wheel 3 Speed", () -> getSwerveModuleStates()[3].speedMetersPerSecond));
  }
}
