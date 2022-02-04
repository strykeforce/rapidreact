// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.SmartDashboardConstants;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.intake.IntakeOpenLoopCommand;
import frc.robot.commands.intake.PitIntakeOpenLoopCommand;
import frc.robot.commands.magazine.PitClearCargoColor;
import frc.robot.commands.magazine.PitMagazineOpenLoopCommand;
import frc.robot.commands.magazine.PitReadCargoColor;
import frc.robot.commands.magazine.UpperMagazineOpenLoopCommand;
import frc.robot.commands.sequences.AutoIntakeCommand;
import frc.robot.commands.shooter.HoodOpenLoopCommand;
import frc.robot.commands.shooter.PitHoodOpenLoopCommand;
import frc.robot.commands.shooter.PitShooterOpenLoopCommand;
import frc.robot.commands.shooter.ShooterOpenLoopCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final MagazineSubsystem magazineSubsystem = new MagazineSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private TelemetryService telemetryService = new TelemetryService(TelemetryController::new);
  private Joystick driveJoystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    driveSubsystem.registerWith(telemetryService);
    shooterSubsystem.registerWith(telemetryService);
    magazineSubsystem.registerWith(telemetryService);
    intakeSubsystem.registerWith(telemetryService);
    telemetryService.start();
    // Configure the button bindings
    configureDriverButtonBindings();
    configurePitDashboard();
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
    new JoystickButton(driveJoystick, Button.HAMBURGER.id)
        .whenPressed(new DriveAutonCommand(driveSubsystem, "straightPath", 0.0));
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

    // Shooter Commands
    SmartDashboard.putNumber(SmartDashboardConstants.kPitShooterOpenLoop, 0.0);
    SmartDashboard.putData(
        "Pit/Shooter/shooterStart", new PitShooterOpenLoopCommand(shooterSubsystem));
    SmartDashboard.putData(
        "Pit/Shooter/shooterStop", new ShooterOpenLoopCommand(shooterSubsystem, 0.0));

    // Hood Commands
    SmartDashboard.putNumber(SmartDashboardConstants.kPitHoodOpenLoop, 0.0);
    SmartDashboard.putData("Pit/Hood/hoodStart", new PitHoodOpenLoopCommand(shooterSubsystem));
    SmartDashboard.putData("Pit/Hood/hoodStop", new HoodOpenLoopCommand(shooterSubsystem, 0.0));
    // intake pit commands
    SmartDashboard.putNumber("Pit/Intake/Speed", 0.0);
    SmartDashboard.putData("Pit/Intake/Start", new PitIntakeOpenLoopCommand(intakeSubsystem));
    SmartDashboard.putData("Pit/Intake/Stop", new IntakeOpenLoopCommand(intakeSubsystem, 0.0));

    SmartDashboard.putData(
        "Temp/AutoIntake", new AutoIntakeCommand(magazineSubsystem, intakeSubsystem));
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

    private final int id;

    Shoulder(int id) {
      this.id = id;
    }
  }

  public enum Toggle {
    LEFT_TOGGLE(1);

    private final int id;

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

    private final int id;

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

    private final int id;

    Trim(int id) {
      this.id = id;
    }
  }
}
