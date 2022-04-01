// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ch.qos.logback.classic.util.ContextInitializer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MagazineConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.HealthCheckCommand;
import frc.robot.commands.climb.EmergencyStopClimbCommand;
import frc.robot.commands.climb.OpenLoopFixedArmCommand;
import frc.robot.commands.climb.OpenLoopPivotArmCommand;
import frc.robot.commands.climb.RotateShoulderDownCommand;
import frc.robot.commands.climb.RotateShoulderUpCommand;
import frc.robot.commands.climb.ShoulderHoldPositionCommand;
import frc.robot.commands.climb.ToggleFixedRatchetCommand;
import frc.robot.commands.climb.TogglePivotRatchetCommand;
import frc.robot.commands.climb.ZeroClimbCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.LockZeroCommand;
import frc.robot.commands.drive.OdometryTestSetPosition;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.drive.XLockCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.intake.IntakeOpenLoopCommand;
import frc.robot.commands.magazine.IgnoreColorSensorCommand;
import frc.robot.commands.magazine.LowerMagazineOpenLoopCommand;
import frc.robot.commands.magazine.ManualEjectCargoReverseCommand;
import frc.robot.commands.magazine.PitClearCargoColor;
import frc.robot.commands.magazine.PitReadCargoColor;
import frc.robot.commands.magazine.StopMagazineCommand;
import frc.robot.commands.magazine.UpperMagazineOpenLoopCommand;
import frc.robot.commands.sequences.climb.HighClimbCommandGroup;
import frc.robot.commands.sequences.climb.MidClimbCommandGroup;
import frc.robot.commands.sequences.climb.TraverseClimbCommandGroup;
import frc.robot.commands.sequences.intaking.AutoIntakeCommand;
import frc.robot.commands.sequences.intaking.ExtendIntakeCommand;
import frc.robot.commands.sequences.shooting.ArmShooterCommandGroup;
import frc.robot.commands.sequences.shooting.HighFenderShotCommand;
import frc.robot.commands.sequences.shooting.LowFenderShotCommand;
import frc.robot.commands.sequences.shooting.PitShooterTuneCommandGroup;
import frc.robot.commands.sequences.shooting.StopShooterCommandGroup;
import frc.robot.commands.sequences.shooting.VisionShootCommand;
import frc.robot.commands.shooter.HoodClosedLoopCommand;
import frc.robot.commands.shooter.HoodOpenLoopCommand;
import frc.robot.commands.shooter.ShooterOpenLoopCommand;
import frc.robot.commands.turret.OpenLoopTurretCommand;
import frc.robot.commands.turret.RotateToCommand;
import frc.robot.commands.turret.TurretAimCommandGroup;
import frc.robot.commands.vision.EnableVisionCommand;
import frc.robot.subsystems.AutoSwitch;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.CargoColor;
import frc.robot.subsystems.OdometryTestSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.Map;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private boolean haveZeroedClimb = false;
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final ClimbSubsystem climbSubsystem;
  private final MagazineSubsystem magazineSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final OdometryTestSubsystem odometryTestSubsystem;
  //   private final PowerDistHub powerDistHub = new PowerDistHub();
  private final AutoSwitch autoSwitch;
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  private final Joystick driveJoystick = new Joystick(0);
  private final XboxController xboxController = new XboxController(1);

  // Dashboard Widgets
  private SuppliedValueWidget<Boolean> firstCargo;
  private SuppliedValueWidget<Boolean> secondCargo;
  private SuppliedValueWidget<Boolean> allianceColor;
  private Alliance alliance = Alliance.Invalid;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DigitalInput eventFlag = new DigitalInput(DashboardConstants.kLockoutBNCid);
    boolean isEvent = eventFlag.get();
    if (isEvent) {
      // must be set before the first call to  LoggerFactory.getLogger();
      System.setProperty(ContextInitializer.CONFIG_FILE_PROPERTY, "logback-event.xml");
      System.out.println("Event Flag Removed - logging to file in ~lvuser/logs/");
    }
    driveSubsystem = new DriveSubsystem();
    visionSubsystem = new VisionSubsystem();
    turretSubsystem = new TurretSubsystem(visionSubsystem, driveSubsystem);
    climbSubsystem = new ClimbSubsystem();
    magazineSubsystem = new MagazineSubsystem(turretSubsystem);
    shooterSubsystem = new ShooterSubsystem(magazineSubsystem, visionSubsystem);
    intakeSubsystem = new IntakeSubsystem();
    odometryTestSubsystem = new OdometryTestSubsystem();
    autoSwitch =
        new AutoSwitch(
            driveSubsystem,
            intakeSubsystem,
            magazineSubsystem,
            turretSubsystem,
            shooterSubsystem,
            visionSubsystem);

    turretSubsystem.setMagazineSubsystem(magazineSubsystem);
    magazineSubsystem.setShooterSubsystem(shooterSubsystem);
    if (!isEvent) {
      configureTelemetry();
      configurePitDashboard();
    }
    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureMatchDashboard();
    // configureManualClimbButtons();
  }

  public VisionSubsystem getVisionSubsystem() {
    return visionSubsystem;
  }

  public AutoSwitch getAutoSwitch() {
    return autoSwitch;
  }

  public void zeroClimb() {
    if (!haveZeroedClimb) {
      climbSubsystem.zeroClimb();
      haveZeroedClimb = true;
    }
  }

  private void configureTelemetry() {
    driveSubsystem.registerWith(telemetryService);
    shooterSubsystem.registerWith(telemetryService);
    magazineSubsystem.registerWith(telemetryService);
    turretSubsystem.registerWith(telemetryService);
    intakeSubsystem.registerWith(telemetryService);
    visionSubsystem.registerWith(telemetryService);
    climbSubsystem.registerWith(telemetryService);
    odometryTestSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriverButtonBindings() {
    driveSubsystem.setDefaultCommand(new DriveTeleopCommand(driveJoystick, driveSubsystem));
    new JoystickButton(driveJoystick, Button.RESET.id)
        .whenPressed(new ZeroGyroCommand(driveSubsystem));
    new JoystickButton(driveJoystick, Button.X.id).whenPressed(new XLockCommand(driveSubsystem));

    // Ignore Color Sensor
    new JoystickButton(driveJoystick, Toggle.LEFT_TOGGLE.id)
        .whenPressed(new IgnoreColorSensorCommand(magazineSubsystem, true));
    new JoystickButton(driveJoystick, Toggle.LEFT_TOGGLE.id)
        .whenReleased(new IgnoreColorSensorCommand(magazineSubsystem, false));

    // Hood Open Loop
    new JoystickButton(driveJoystick, Trim.RIGHT_Y_POS.id)
        .whenPressed(new HoodOpenLoopCommand(shooterSubsystem, 0.2));
    new JoystickButton(driveJoystick, Trim.RIGHT_Y_POS.id)
        .whenReleased(new HoodOpenLoopCommand(shooterSubsystem, 0.0));
    new JoystickButton(driveJoystick, Trim.RIGHT_Y_NEG.id)
        .whenPressed(new HoodOpenLoopCommand(shooterSubsystem, -0.2));
    new JoystickButton(driveJoystick, Trim.RIGHT_Y_NEG.id)
        .whenReleased(new HoodOpenLoopCommand(shooterSubsystem, 0.0));

    // Turret Open Loop
    new JoystickButton(driveJoystick, Trim.RIGHT_X_POS.id)
        .whenPressed(new OpenLoopTurretCommand(turretSubsystem, 0.3));
    new JoystickButton(driveJoystick, Trim.RIGHT_X_POS.id)
        .whenReleased(new OpenLoopTurretCommand(turretSubsystem, 0.0));
    new JoystickButton(driveJoystick, Trim.RIGHT_X_NEG.id)
        .whenPressed(new OpenLoopTurretCommand(turretSubsystem, -0.3));
    new JoystickButton(driveJoystick, Trim.RIGHT_X_NEG.id)
        .whenReleased(new OpenLoopTurretCommand(turretSubsystem, 0.0));

    // new JoystickButton(driveJoystick, Button.HAMBURGER.id)
    //     .whenPressed(new DriveAutonCommand(driveSubsystem, "straightPath", true, true));

    new JoystickButton(driveJoystick, Trim.LEFT_Y_POS.id)
        .whenPressed(new EnableVisionCommand(visionSubsystem));

    // Drive Practice Odometry Reset
    new JoystickButton(driveJoystick, Trim.LEFT_Y_NEG.id)
        .whenPressed(
            new ResetOdometryCommand(
                driveSubsystem,
                new Pose2d(new Translation2d(0.46, 4.11), Rotation2d.fromDegrees(0)))); // Y:7.42

    // Auto Intake
    new JoystickButton(driveJoystick, Shoulder.LEFT_DOWN.id)
        .whenPressed(new AutoIntakeCommand(magazineSubsystem, intakeSubsystem, false, false));
    new JoystickButton(driveJoystick, Shoulder.LEFT_DOWN.id)
        .whenReleased(new IntakeOpenLoopCommand(intakeSubsystem, 0.0));

    // Vision Shoot
    new JoystickButton(driveJoystick, Shoulder.RIGHT_DOWN.id)
        .whenPressed(
            new VisionShootCommand(
                shooterSubsystem,
                turretSubsystem,
                magazineSubsystem,
                visionSubsystem,
                true,
                intakeSubsystem));

    // Auto Climb
    new JoystickButton(driveJoystick, Trim.LEFT_X_POS.id)
        .whenPressed(
            new TraverseClimbCommandGroup(
                climbSubsystem, driveSubsystem, driveJoystick, turretSubsystem));
    new JoystickButton(driveJoystick, Trim.LEFT_X_NEG.id)
        .whenPressed(
            new TraverseClimbCommandGroup(
                climbSubsystem, driveSubsystem, driveJoystick, turretSubsystem));
    new JoystickButton(driveJoystick, Button.UP.id)
        .whenPressed(
            new HighClimbCommandGroup(
                climbSubsystem, driveSubsystem, driveJoystick, turretSubsystem));
    new JoystickButton(driveJoystick, Button.DOWN.id)
        .whenPressed(
            new MidClimbCommandGroup(
                climbSubsystem, driveSubsystem, driveJoystick, turretSubsystem));
  }

  private void configureOperatorButtonBindings() {
    // Zero Climb
    new JoystickButton(xboxController, XboxController.Button.kStart.value)
        .whenReleased(new ZeroClimbCommand(climbSubsystem));

    // Auto Intake
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .toggleWhenPressed(new AutoIntakeCommand(magazineSubsystem, intakeSubsystem, false, true));
    LeftTriggerDown.whileActiveOnce(new ExtendIntakeCommand(intakeSubsystem, true));
    RightTriggerDown.whileActiveOnce(new ExtendIntakeCommand(intakeSubsystem, false));

    // Eject Cargo Reverse
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
        .whenPressed(new ManualEjectCargoReverseCommand(magazineSubsystem, intakeSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
        .whenReleased(
            new ParallelCommandGroup(
                new IntakeOpenLoopCommand(intakeSubsystem, 0.0),
                new StopMagazineCommand(magazineSubsystem)));

    // Arm Shooter
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .whenPressed(
            new ArmShooterCommandGroup(visionSubsystem, turretSubsystem, shooterSubsystem));

    // Stop Shoot
    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .whenPressed(
            new StopShooterCommandGroup(
                magazineSubsystem,
                visionSubsystem,
                turretSubsystem,
                shooterSubsystem,
                intakeSubsystem));

    // Eject Opponent Cargo
    // new JoystickButton(xboxController, XboxController.Button.kA.value)
    //     .whenPressed(
    //         new EjectCargoCommand(
    //             turretSubsystem, shooterSubsystem, magazineSubsystem, intakeSubsystem));
    // new JoystickButton(xboxController, XboxController.Button.kA.value)
    //     .whenReleased(new StopShooterCommand(shooterSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .toggleWhenPressed(new AutoIntakeCommand(magazineSubsystem, intakeSubsystem, false, false));

    // High Fender Shot
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .whenPressed(
            new HighFenderShotCommand(
                turretSubsystem, shooterSubsystem, magazineSubsystem, intakeSubsystem));

    // Low Fender Shot
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .whenPressed(
            new LowFenderShotCommand(
                turretSubsystem, shooterSubsystem, magazineSubsystem, intakeSubsystem));
  }

  public void configureManualClimbButtons() {
    // Manual Climb
    // Rotating Arm
    LeftStickUp.whenActive(
        new OpenLoopPivotArmCommand(climbSubsystem, ClimbConstants.kClimbArmsOpenLoopSpeed));
    LeftStickDown.whenActive(
        new OpenLoopPivotArmCommand(climbSubsystem, -ClimbConstants.kClimbArmsOpenLoopSpeed));
    LeftStickStop.whenActive(new OpenLoopPivotArmCommand(climbSubsystem, 0.0));
    new JoystickButton(xboxController, XboxController.Button.kLeftStick.value)
        .whenPressed(new TogglePivotRatchetCommand(climbSubsystem));

    // // Static Arm
    RightStickUp.whenActive(
        new OpenLoopFixedArmCommand(climbSubsystem, ClimbConstants.kClimbArmsOpenLoopSpeed));
    RightStickDown.whenActive(
        new OpenLoopFixedArmCommand(climbSubsystem, -ClimbConstants.kClimbArmsOpenLoopSpeed));
    RightStickStop.whenActive(new OpenLoopFixedArmCommand(climbSubsystem, 0.0));
    new JoystickButton(xboxController, XboxController.Button.kRightStick.value)
        .whenPressed(new ToggleFixedRatchetCommand(climbSubsystem));

    // // Shoulder
    LeftTriggerDown.whenActive(new RotateShoulderUpCommand(climbSubsystem));
    LeftTriggerDown.whenInactive(new ShoulderHoldPositionCommand(climbSubsystem));
    RightTriggerDown.whenActive(new RotateShoulderDownCommand(climbSubsystem));
    RightTriggerDown.whenInactive(new ShoulderHoldPositionCommand(climbSubsystem));
  }

  private void configureMatchDashboard() {
    firstCargo =
        Shuffleboard.getTab("Match")
            .addBoolean(
                "First Cargo", () -> magazineSubsystem.getAllCargoColors()[0] != CargoColor.NONE)
            .withProperties(Map.of("colorWhenFalse", "black"))
            .withSize(2, 1)
            .withPosition(5, 0);
    secondCargo =
        Shuffleboard.getTab("Match")
            .addBoolean(
                "Second Cargo", () -> magazineSubsystem.getAllCargoColors()[1] != CargoColor.NONE)
            .withProperties(Map.of("colorWhenFalse", "black"))
            .withPosition(5, 1)
            .withSize(2, 1);

    allianceColor =
        Shuffleboard.getTab("Match")
            .addBoolean("AllianceColor", () -> alliance != Alliance.Invalid)
            .withProperties(Map.of("colorWhenFalse", "black"))
            .withSize(2, 2)
            .withPosition(0, 0);

    Shuffleboard.getTab("Match")
        .addBoolean("IgnoreColorSensor", () -> magazineSubsystem.isColorSensorIgnored())
        .withSize(2, 2)
        .withPosition(3, 0);

    Shuffleboard.getTab("Match")
        .add("EstopClimb", new EmergencyStopClimbCommand(climbSubsystem))
        .withSize(2, 2)
        .withPosition(7, 0);
  }

  public void setAllianceColor(Alliance alliance) {
    this.alliance = alliance;
    allianceColor.withProperties(
        Map.of(
            "colorWhenTrue", alliance == Alliance.Red ? "red" : "blue", "colorWhenFalse", "black"));
    magazineSubsystem.setAllianceColor(alliance);
  }

  public void updateMatchData() {
    firstCargo.withProperties(
        Map.of(
            "colorWhenTrue",
            magazineSubsystem.getAllCargoColors()[0].color,
            "colorWhenFalse",
            "black"));
    secondCargo.withProperties(
        Map.of(
            "colorWhenTrue",
            magazineSubsystem.getAllCargoColors()[1].color,
            "colorWhenFalse",
            "black"));
  }

  private void configurePitDashboard() {
    ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");

    // Magazine Commands
    ShuffleboardLayout magazineCommands =
        pitTab.getLayout("Magazine", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(1, 3);
    magazineCommands
        .add("Clear Cargo", new PitClearCargoColor(magazineSubsystem))
        .withPosition(0, 0);
    magazineCommands
        .add("Read Cargo Color", new PitReadCargoColor(magazineSubsystem))
        .withPosition(0, 1);
    magazineCommands.add("Stop", new StopMagazineCommand(magazineSubsystem)).withPosition(0, 2);
    magazineCommands
        .add(
            "Start",
            new SequentialCommandGroup(
                new UpperMagazineOpenLoopCommand(magazineSubsystem, 0.5),
                new LowerMagazineOpenLoopCommand(magazineSubsystem, 0.5)))
        .withPosition(0, 3);

    // SmartDashboard.putNumber("Pit/Magazine/Speed", 0.0);
    // SmartDashboard.putData("Pit/Magazine/Start", new
    // PitMagazineOpenLoopCommand(magazineSubsystem));
    // SmartDashboard.putData(
    //     "Pit/Magazine/Stop", new UpperMagazineOpenLoopCommand(magazineSubsystem, 0.0));
    // SmartDashboard.putString("Pit/Magazine/First Cargo Color", "");
    // SmartDashboard.putString("Pit/Magazine/Second Cargo Color", "");
    // SmartDashboard.putData("Pit/Magazine/ReadCargoColor", new
    // PitReadCargoColor(magazineSubsystem));
    // SmartDashboard.putData(
    //     "Pit/Magazine/ClearCargoColor", new PitClearCargoColor(magazineSubsystem));

    // Drive Commands
    ShuffleboardLayout driveCommands =
        pitTab.getLayout("Drive", BuiltInLayouts.kGrid).withSize(2, 3).withPosition(7, 0);
    driveCommands.add("LockZero", new LockZeroCommand(driveSubsystem)).withPosition(0, 0);
    driveCommands
        .add("OdometryTuning", new DriveAutonCommand(driveSubsystem, "straightPath", true, false))
        .withPosition(0, 1);
    driveCommands
        .add("SetOdometry: 1", new OdometryTestSetPosition(odometryTestSubsystem, 1))
        .withPosition(0, 2);
    driveCommands
        .add("SetOdometry: 2", new OdometryTestSetPosition(odometryTestSubsystem, 2))
        .withPosition(1, 0);
    driveCommands
        .add("SetOdometry: 3", new OdometryTestSetPosition(odometryTestSubsystem, 3))
        .withPosition(1, 1);
    driveCommands
        .add("SetOdometry: 4", new OdometryTestSetPosition(odometryTestSubsystem, 4))
        .withPosition(1, 2);

    // SmartDashboard.putNumber("Pit/Drive/PoseX", 8.42);
    // SmartDashboard.putNumber("Pit/Drive/PoseY", 7.89);
    // SmartDashboard.putData(
    //     "Pit/Drive/Set Odometry",
    //     new ResetOdometryCommand(
    //         driveSubsystem,
    //         new Pose2d(
    //             SmartDashboard.getNumber("Pit/Drive/PoseX", 8.42),
    //             SmartDashboard.getNumber("Pit/Drive/PoseY", 7.89),
    //             Rotation2d.fromDegrees(0))));
    // SmartDashboard.putData("Pit/Drive/LockZero", new LockZeroCommand(driveSubsystem));
    // SmartDashboard.putData(
    //     "Pit/Drive/pathDrive", new DriveAutonCommand(driveSubsystem, "straightPath"));

    // Shooter (shooter, kicker, hood) Commands
    ShuffleboardLayout shooterCommands =
        pitTab.getLayout("Shooter", BuiltInLayouts.kGrid).withPosition(1, 0).withSize(1, 2);
    shooterCommands
        .add("Stop", new ShooterOpenLoopCommand(shooterSubsystem, 0.0))
        .withPosition(0, 0);
    shooterCommands
        .add("HoodToZero", new HoodClosedLoopCommand(shooterSubsystem, 0.0))
        .withPosition(0, 1);
    shooterCommands
        .add(
            "HoodZeroCheck",
            new HoodClosedLoopCommand(shooterSubsystem, ShooterConstants.kZeroCheckTicks))
        .withPosition(0, 2);

    SmartDashboard.putNumber(DashboardConstants.kPitShooterSetpointTicks, 0.0);
    SmartDashboard.putNumber(DashboardConstants.kPitKickerSetpointTicks, 0.0);
    // SmartDashboard.putData(
    //     "Pit/Shooter/shooterStart", new PitShooterClosedLoopCommand(shooterSubsystem));
    // SmartDashboard.putData(
    //     "Pit/Shooter/shooterStop", new ShooterOpenLoopCommand(shooterSubsystem, 0.0));

    // // Hood Commands
    SmartDashboard.putNumber(DashboardConstants.kPitHoodSetpointTicks, 0.0);
    // SmartDashboard.putData("Pit/Hood/hoodStart", new PitHoodClosedLoopCommand(shooterSubsystem));
    // SmartDashboard.putData("Pit/Hood/hoodStop", new HoodOpenLoopCommand(shooterSubsystem, 0.0));

    // Turret Pit Commands
    ShuffleboardLayout turretCommands =
        pitTab.getLayout("Turret", BuiltInLayouts.kGrid).withPosition(2, 0).withSize(1, 2);
    turretCommands
        .add("LockZero", new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(0.0)))
        .withPosition(0, 0);
    turretCommands
        .add("Lock90", new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(90.0)))
        .withPosition(0, 1);
    turretCommands
        .add("LockNeg90", new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(-90.0)))
        .withPosition(0, 2);

    // SmartDashboard.putNumber(
    //     DashboardConstants.kTurretSetpointRadians, turretSubsystem.getRotation2d().getRadians());
    // SmartDashboard.putData(
    //     "Pit/Turret/CloseLoopPosition", new PitTurretCloseLoopPositionCommand(turretSubsystem));
    // SmartDashboard.putData("Pit/Turret/Forward", new OpenLoopTurretCommand(turretSubsystem,
    // 0.3));
    // SmartDashboard.putData("Pit/Turret/Reverse", new OpenLoopTurretCommand(turretSubsystem,
    // -0.3));
    // SmartDashboard.putData("Pit/Turret/Stop", new OpenLoopTurretCommand(turretSubsystem, 0.0));
    // SmartDashboard.putData(
    //     "Pit/Turret/RotateToPos90",
    //     new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(90)));
    // SmartDashboard.putData(
    //     "Pit/Turret/RotateTo0", new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(0)));
    // SmartDashboard.putData(
    //     "Pit/Turret/RotateToNeg90",
    //     new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(-90)));

    // Intake Commands
    ShuffleboardLayout intakeCommands =
        pitTab.getLayout("Intake", BuiltInLayouts.kGrid).withPosition(3, 0).withSize(1, 2);
    intakeCommands
        .add("FWD", new IntakeOpenLoopCommand(intakeSubsystem, IntakeConstants.kIntakeSpeed))
        .withPosition(0, 0);
    intakeCommands
        .add("REV", new IntakeOpenLoopCommand(intakeSubsystem, IntakeConstants.kIntakeEjectSpeed))
        .withPosition(0, 1);
    intakeCommands.add("Stop", new IntakeOpenLoopCommand(intakeSubsystem, 0.0)).withPosition(0, 2);
    intakeCommands.add("Extend", new ExtendIntakeCommand(intakeSubsystem, true)).withPosition(0, 3);
    intakeCommands
        .add("Retract", new ExtendIntakeCommand(intakeSubsystem, false))
        .withPosition(0, 4);

    // SmartDashboard.putData("Pit/Intake/Start", new PitIntakeOpenLoopCommand(intakeSubsystem));
    // SmartDashboard.putData("Pit/Intake/Stop", new IntakeOpenLoopCommand(intakeSubsystem, 0.0));
    // SmartDashboard.putNumber("Pit/Intake/Speed", 0.0);

    // Climb Commands
    ShuffleboardLayout climbCommands =
        pitTab.getLayout("Climb", BuiltInLayouts.kGrid).withPosition(4, 0).withSize(1, 2);
    climbCommands.add("Zero", new ZeroClimbCommand(climbSubsystem)).withPosition(0, 0);
    climbCommands
        .add("ToggleFixedRatchet", new ToggleFixedRatchetCommand(climbSubsystem))
        .withPosition(0, 0);
    climbCommands
        .add("TogglePivotRatchet", new TogglePivotRatchetCommand(climbSubsystem))
        .withPosition(0, 0);
    // climbCommands.add(
    //     "FixedArmExtend",
    //     new OpenLoopFixedArmCommand(climbSubsystem, -ClimbConstants.kClimbArmsOpenLoopSpeed));
    // climbCommands.add(
    //     "FixedArmRetract",
    //     new OpenLoopFixedArmCommand(climbSubsystem, ClimbConstants.kClimbArmsOpenLoopSpeed));
    // climbCommands.add("FixedArmsStop", new OpenLoopFixedArmCommand(climbSubsystem, 0.0));
    // climbCommands.add(
    //     "PivotArmsExtend",
    //     new OpenLoopPivotArmCommand(climbSubsystem, -ClimbConstants.kClimbArmsOpenLoopSpeed));
    // climbCommands.add(
    //     "PivotArmsRetract",
    //     new OpenLoopPivotArmCommand(climbSubsystem, ClimbConstants.kClimbArmsOpenLoopSpeed));
    // climbCommands.add("PivotArmsStop", new OpenLoopPivotArmCommand(climbSubsystem, 0.0));
    // climbCommands.add("ShoulderFWD", new RotateShoulderDownCommand(climbSubsystem));
    // climbCommands.add("ShoulderREV", new RotateShoulderUpCommand(climbSubsystem));
    // climbCommands.add("ShoulderSTOP", new ShoulderHoldPositionCommand(climbSubsystem));

    // Vision Commands
    pitTab
        .add("Seek", new TurretAimCommandGroup(visionSubsystem, turretSubsystem))
        .withPosition(5, 0);
    // SmartDashboard.putData(
    //     "Pit/Turret/Seek", new TurretAimCommandGroup(visionSubsystem, turretSubsystem));

    // HealthCheck
    pitTab
        .add(
            "HealthCheck",
            new HealthCheckCommand(
                driveSubsystem,
                magazineSubsystem,
                intakeSubsystem,
                shooterSubsystem,
                turretSubsystem))
        .withPosition(6, 0);

    // tuning commands
    SmartDashboard.putData(
        "Tune/Start",
        new PitShooterTuneCommandGroup(
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            turretSubsystem,
            visionSubsystem));
    SmartDashboard.putData(
        "Tune/Stop",
        new StopShooterCommandGroup(
            magazineSubsystem,
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            intakeSubsystem));
    SmartDashboard.putNumber(
        DashboardConstants.kTuneUpperMagSpeedTicks, MagazineConstants.kUpperMagazineIntakeSpeed);

    // Power Cycle Deadeye
    // SmartDashboard.putData("Pit/Deadeye/PowerCycle", new PowerCycleDeadeyeCommand(powerDistHub));
  }

  public enum Axis {
    RIGHT_X(1),
    RIGHT_Y(0),
    LEFT_X(2),
    LEFT_Y(5),
    TUNER(6),
    LEFT_BACK(4),
    RIGHT_BACK(3);

    public final int id;

    Axis(int id) {
      this.id = id;
    }
  }

  public enum Shoulder {
    RIGHT_DOWN(2),
    LEFT_DOWN(4),
    LEFT_UP(5);

    public final int id;

    Shoulder(int id) {
      this.id = id;
    }
  }

  public enum Toggle {
    LEFT_TOGGLE(1);

    public final int id;

    Toggle(int id) {
      this.id = id;
    }
  }

  public enum Button {
    RESET(3),
    HAMBURGER(14),
    X(15),
    UP(16),
    DOWN(17);

    public final int id;

    Button(int id) {
      this.id = id;
    }
  }

  public enum Trim {
    LEFT_Y_POS(7),
    LEFT_Y_NEG(6),
    LEFT_X_POS(8),
    LEFT_X_NEG(9),
    RIGHT_X_POS(10),
    RIGHT_X_NEG(11),
    RIGHT_Y_POS(12),
    RIGHT_Y_NEG(13);

    public final int id;

    Trim(int id) {
      this.id = id;
    }
  }

  Trigger LeftStickUp =
      new Trigger() {
        @Override
        public boolean get() {
          return xboxController.getRawAxis(XboxController.Axis.kLeftY.value)
              > DashboardConstants.kLeftStickDeadBand;
        }
      };

  Trigger LeftStickDown =
      new Trigger() {
        @Override
        public boolean get() {
          return xboxController.getRawAxis(XboxController.Axis.kLeftY.value)
              < -DashboardConstants.kLeftStickDeadBand;
        }
      };

  Trigger LeftStickStop =
      new Trigger() {
        @Override
        public boolean get() {
          return Math.abs(xboxController.getRawAxis(XboxController.Axis.kLeftY.value))
              < DashboardConstants.kLeftStickDeadBand;
        }
      };

  Trigger RightStickUp =
      new Trigger() {
        @Override
        public boolean get() {
          return xboxController.getRawAxis(XboxController.Axis.kRightY.value)
              > DashboardConstants.kRightStickDeadBand;
        }
      };

  Trigger RightStickDown =
      new Trigger() {
        @Override
        public boolean get() {
          return xboxController.getRawAxis(XboxController.Axis.kRightY.value)
              < -DashboardConstants.kRightStickDeadBand;
        }
      };

  Trigger RightStickStop =
      new Trigger() {
        @Override
        public boolean get() {
          return Math.abs(xboxController.getRawAxis(XboxController.Axis.kRightY.value))
              < DashboardConstants.kRightStickDeadBand;
        }
      };

  Trigger LeftTriggerDown =
      new Trigger() {
        @Override
        public boolean get() {
          return xboxController.getRawAxis(XboxController.Axis.kLeftTrigger.value)
              > DashboardConstants.kTriggerDeadBand;
        }
      };

  Trigger RightTriggerDown =
      new Trigger() {
        @Override
        public boolean get() {
          return xboxController.getRawAxis(XboxController.Axis.kRightTrigger.value)
              > DashboardConstants.kTriggerDeadBand;
        }
      };
}
