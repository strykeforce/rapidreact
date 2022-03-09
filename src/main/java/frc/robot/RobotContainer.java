// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.MagazineConstants;
import frc.robot.commands.Auton.TwoCargoAuto;
import frc.robot.commands.climb.OpenLoopFixedArmCommand;
import frc.robot.commands.climb.OpenLoopPivotArmCommand;
import frc.robot.commands.climb.RotateShoulderDownCommand;
import frc.robot.commands.climb.RotateShoulderUpCommand;
import frc.robot.commands.climb.ShoulderHoldPositionCommand;
import frc.robot.commands.climb.ToggleFixedRatchetCommand;
import frc.robot.commands.climb.TogglePivotRatchetCommand;
import frc.robot.commands.climb.ZeroMotorsCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.LockZeroCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.drive.XLockCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.intake.IntakeOpenLoopCommand;
import frc.robot.commands.intake.PitIntakeOpenLoopCommand;
import frc.robot.commands.magazine.IgnoreColorSensorCommand;
import frc.robot.commands.magazine.ManualEjectCargoReverseCommand;
import frc.robot.commands.magazine.PitClearCargoColor;
import frc.robot.commands.magazine.PitMagazineOpenLoopCommand;
import frc.robot.commands.magazine.PitReadCargoColor;
import frc.robot.commands.magazine.StopMagazineCommand;
import frc.robot.commands.magazine.UpperMagazineOpenLoopCommand;
import frc.robot.commands.sequences.ArmShooterCommandGroup;
import frc.robot.commands.sequences.AutoIntakeCommand;
import frc.robot.commands.sequences.EjectCargoCommand;
import frc.robot.commands.sequences.HighFenderShotCommand;
import frc.robot.commands.sequences.LowFenderShotCommand;
import frc.robot.commands.sequences.MidClimbCommandGroup;
import frc.robot.commands.sequences.PitShooterTuneCommandGroup;
import frc.robot.commands.sequences.StopShooterCommandGroup;
import frc.robot.commands.sequences.VisionShootCommand;
import frc.robot.commands.shooter.HoodOpenLoopCommand;
import frc.robot.commands.shooter.PitHoodClosedLoopCommand;
import frc.robot.commands.shooter.PitShooterClosedLoopCommand;
import frc.robot.commands.shooter.ShooterOpenLoopCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.commands.turret.CheckSeekAngleCommand;
import frc.robot.commands.turret.OpenLoopTurretCommand;
import frc.robot.commands.turret.PitTurretCloseLoopPositionCommand;
import frc.robot.commands.turret.RotateToCommand;
import frc.robot.commands.turret.TurretAimCommandGroup;
import frc.robot.subsystems.AutoSwitch;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.CargoColor;
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
  private boolean isEvent = true;
  private DigitalInput eventFlag = new DigitalInput(DashboardConstants.kLockoutBNCid);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final TurretSubsystem turretSubsystem =
      new TurretSubsystem(visionSubsystem, driveSubsystem);
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final MagazineSubsystem magazineSubsystem = new MagazineSubsystem(turretSubsystem);
  private final ShooterSubsystem shooterSubsystem =
      new ShooterSubsystem(magazineSubsystem, visionSubsystem);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  //   private final PowerDistHub powerDistHub = new PowerDistHub();
  private final AutoSwitch autoSwitch =
      new AutoSwitch(
          driveSubsystem,
          intakeSubsystem,
          magazineSubsystem,
          turretSubsystem,
          shooterSubsystem,
          visionSubsystem);
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  private final Joystick driveJoystick = new Joystick(0);
  private final XboxController xboxController = new XboxController(1);

  // Dashboard Widgets
  private SuppliedValueWidget firstCargo;
  private SuppliedValueWidget secondCargo;
  private SuppliedValueWidget allianceColor;
  private Alliance alliance = Alliance.Invalid;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // isEvent = eventFlag.get();
    // if(isEvent){
    //     System.out.println("Event Flag Removed - switching to log file");
    //     System.out.property(ContextInitializer.CONFIG_FILE_PROPERTY, "logback-event.xml")
    // }
    turretSubsystem.setMagazineSubsystem(magazineSubsystem);
    magazineSubsystem.setShooterSubsystem(shooterSubsystem);
    configureTelemetry();
    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configurePitDashboard();
    configureMatchDashboard();
    configureManualClimbButtons();
  }

  public VisionSubsystem getVisionSubsystem() {
    return visionSubsystem;
  }

  public AutoSwitch getAutoSwitch() {
    return autoSwitch;
  }

  private void configureTelemetry() {
    driveSubsystem.registerWith(telemetryService);
    shooterSubsystem.registerWith(telemetryService);
    magazineSubsystem.registerWith(telemetryService);
    turretSubsystem.registerWith(telemetryService);
    intakeSubsystem.registerWith(telemetryService);
    visionSubsystem.registerWith(telemetryService);
    climbSubsystem.registerWith(telemetryService);
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
    new JoystickButton(driveJoystick, Trim.LEFT_X_POS.id)
        .whenPressed(new HoodOpenLoopCommand(shooterSubsystem, 0.2));
    new JoystickButton(driveJoystick, Trim.LEFT_X_POS.id)
        .whenReleased(new HoodOpenLoopCommand(shooterSubsystem, 0.0));
    new JoystickButton(driveJoystick, Trim.LEFT_X_NEG.id)
        .whenPressed(new HoodOpenLoopCommand(shooterSubsystem, -0.2));
    new JoystickButton(driveJoystick, Trim.LEFT_X_NEG.id)
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
    new JoystickButton(driveJoystick, Trim.RIGHT_Y_POS.id)
        .whenPressed(new CheckSeekAngleCommand(turretSubsystem));
    new JoystickButton(driveJoystick, Trim.LEFT_Y_NEG.id)
        .whenPressed(
            new ResetOdometryCommand(
                driveSubsystem,
                new Pose2d(new Translation2d(0.415, 7.42), Rotation2d.fromDegrees(0))));

    // Auto Intake
    new JoystickButton(driveJoystick, Shoulder.LEFT_DOWN.id)
        .whenPressed(new AutoIntakeCommand(magazineSubsystem, intakeSubsystem));
    new JoystickButton(driveJoystick, Shoulder.LEFT_DOWN.id)
        .whenReleased(new IntakeOpenLoopCommand(intakeSubsystem, 0.0));

    // Vision Shoot
    new JoystickButton(driveJoystick, Shoulder.RIGHT_DOWN.id)
        .whenPressed(
            new VisionShootCommand(
                shooterSubsystem, turretSubsystem, magazineSubsystem, visionSubsystem, true));
    // new JoystickButton(driveJoystick, Shoulder.RIGHT_DOWN.id)
    //     .whenReleased(new StopShooterCommand(shooterSubsystem));

    // Auto Climb
    // new JoystickButton(driveJoystick, Button.UP.id)
    //     .whenPressed(new TraverseClimbCommandGroup(climbSubsystem, driveSubsystem,
    // driveJoystick));
    // new JoystickButton(driveJoystick, Button.DOWN.id)
    //     .whenPressed(new HighClimbCommandGroup(climbSubsystem, driveSubsystem, driveJoystick));
    // new JoystickButton(driveJoystick, Button.HAMBURGER.id)
    //     .whenPressed(new MidClimbCommandGroup(climbSubsystem, driveSubsystem, driveJoystick));
    new JoystickButton(driveJoystick, Button.UP.id)
        .whenPressed(new MidClimbCommandGroup(climbSubsystem, driveSubsystem, driveJoystick));
    new JoystickButton(driveJoystick, Button.HAMBURGER.id)
        .whenPressed(new DriveAutonCommand(driveSubsystem, "LeftCargoCollect"));
    new JoystickButton(driveJoystick, Button.DOWN.id)
        .whenPressed(
            new TwoCargoAuto(
                visionSubsystem,
                turretSubsystem,
                shooterSubsystem,
                magazineSubsystem,
                intakeSubsystem,
                driveSubsystem,
                "MidCargo1Collect",
                Rotation2d.fromDegrees(-156)));
  }

  private void configureOperatorButtonBindings() {
    // Zero Climb
    new JoystickButton(xboxController, XboxController.Button.kStart.value)
        .whenReleased(new ZeroMotorsCommand(climbSubsystem));

    // Auto Intake
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .toggleWhenPressed(new AutoIntakeCommand(magazineSubsystem, intakeSubsystem));

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
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .whenPressed(new EjectCargoCommand(turretSubsystem, shooterSubsystem, magazineSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .whenReleased(new StopShooterCommand(shooterSubsystem));

    // High Fender Shot
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .whenPressed(
            new HighFenderShotCommand(turretSubsystem, shooterSubsystem, magazineSubsystem));

    // Low Fender Shot
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .whenPressed(
            new LowFenderShotCommand(turretSubsystem, shooterSubsystem, magazineSubsystem));
  }

  private void configureManualClimbButtons() {
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
            .withProperties(Map.of("colorWhenFalse", "black"));
    secondCargo =
        Shuffleboard.getTab("Match")
            .addBoolean(
                "Second Cargo", () -> magazineSubsystem.getAllCargoColors()[1] != CargoColor.NONE)
            .withProperties(Map.of("colorWhenFalse", "black"));

    allianceColor =
        Shuffleboard.getTab("Match")
            .addBoolean("AllianceColor", () -> alliance != Alliance.Invalid)
            .withProperties(Map.of("colorWhenFalse", "black"));
  }

  public void setAllianceColor(Alliance alliance) {
    this.alliance = alliance;
    allianceColor.withProperties(
        Map.of(
            "colorWhenTrue", alliance == Alliance.Red ? "red" : "blue", "colorWhenFalse", "black"));
    magazineSubsystem.setAllianceColor(alliance);
  }

  public void upudateMatchData() {
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
    // Magazine Commands
    SmartDashboard.putNumber("Pit/Magazine/Speed", 0.0);
    SmartDashboard.putData("Pit/Magazine/Start", new PitMagazineOpenLoopCommand(magazineSubsystem));
    SmartDashboard.putData(
        "Pit/Magazine/Stop", new UpperMagazineOpenLoopCommand(magazineSubsystem, 0.0));
    SmartDashboard.putString("Pit/Magazine/First Cargo Color", "");
    SmartDashboard.putString("Pit/Magazine/Second Cargo Color", "");
    SmartDashboard.putData("Pit/Magazine/ReadCargoColor", new PitReadCargoColor(magazineSubsystem));
    SmartDashboard.putData(
        "Pit/Magazine/ClearCargoColor", new PitClearCargoColor(magazineSubsystem));

    // Drive Commands
    SmartDashboard.putNumber("Pit/Drive/PoseX", 8.42);
    SmartDashboard.putNumber("Pit/Drive/PoseY", 7.89);
    SmartDashboard.putData(
        "Pit/Drive/Set Odometry",
        new ResetOdometryCommand(
            driveSubsystem,
            new Pose2d(
                SmartDashboard.getNumber("Pit/Drive/PoseX", 8.42),
                SmartDashboard.getNumber("Pit/Drive/PoseY", 7.89),
                Rotation2d.fromDegrees(0))));
    SmartDashboard.putData("Pit/Drive/LockZero", new LockZeroCommand(driveSubsystem));
    SmartDashboard.putData(
        "Pit/Drive/pathDrive", new DriveAutonCommand(driveSubsystem, "straightPath"));

    // Shooter Commands
    SmartDashboard.putNumber(DashboardConstants.kPitShooterSetpointTicks, 0.0);
    SmartDashboard.putNumber(DashboardConstants.kPitKickerSetpointTicks, 0.0);
    SmartDashboard.putData(
        "Pit/Shooter/shooterStart", new PitShooterClosedLoopCommand(shooterSubsystem));
    SmartDashboard.putData(
        "Pit/Shooter/shooterStop", new ShooterOpenLoopCommand(shooterSubsystem, 0.0));

    // Hood Commands
    SmartDashboard.putNumber(DashboardConstants.kPitHoodSetpointTicks, 0.0);
    SmartDashboard.putData("Pit/Hood/hoodStart", new PitHoodClosedLoopCommand(shooterSubsystem));
    SmartDashboard.putData("Pit/Hood/hoodStop", new HoodOpenLoopCommand(shooterSubsystem, 0.0));

    // intake pit commands
    SmartDashboard.putNumber("Pit/Intake/Speed", 0.0);

    // Turret Pit Commands
    SmartDashboard.putNumber(
        DashboardConstants.kTurretSetpointRadians, turretSubsystem.getRotation2d().getRadians());
    SmartDashboard.putData(
        "Pit/Turret/CloseLoopPosition", new PitTurretCloseLoopPositionCommand(turretSubsystem));
    SmartDashboard.putData("Pit/Turret/Forward", new OpenLoopTurretCommand(turretSubsystem, 0.3));
    SmartDashboard.putData("Pit/Turret/Reverse", new OpenLoopTurretCommand(turretSubsystem, -0.3));
    SmartDashboard.putData("Pit/Turret/Stop", new OpenLoopTurretCommand(turretSubsystem, 0.0));
    SmartDashboard.putData(
        "Pit/Turret/RotateToPos90",
        new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(90)));
    SmartDashboard.putData(
        "Pit/Turret/RotateTo0", new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(0)));
    SmartDashboard.putData(
        "Pit/Turret/RotateToNeg90",
        new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(-90)));

    // Intake Commands
    SmartDashboard.putData("Pit/Intake/Start", new PitIntakeOpenLoopCommand(intakeSubsystem));
    SmartDashboard.putData("Pit/Intake/Stop", new IntakeOpenLoopCommand(intakeSubsystem, 0.0));
    SmartDashboard.putData(
        "Pit/Turret/Seek", new TurretAimCommandGroup(visionSubsystem, turretSubsystem));
    // SmartDashboard.putData("Pit/Turret/AimTurret", new TurretAimCommand(visionSubsystem,
    // turretSubsystem));

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
