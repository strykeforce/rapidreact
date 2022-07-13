package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.OffsetGyroCommand;
import frc.robot.commands.magazine.IgnoreColorSensorCommand;
import frc.robot.commands.magazine.PreloadCargoCommand;
import frc.robot.commands.sequences.intaking.AutoIntakeCommand;
import frc.robot.commands.sequences.shooting.ArmShooterCommandGroup;
import frc.robot.commands.sequences.shooting.OppCargoShotAutonCommand;
import frc.robot.commands.sequences.shooting.VisionShootAutoCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DefenseTwoAuto extends SequentialCommandGroup {

  public DefenseTwoAuto(
      VisionSubsystem visionSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeExtendSubsystem intakeExtendSubsystem,
      DriveSubsystem driveSubsystem,
      String path1Name,
      String path2Name,
      Rotation2d gyroOffset,
      double delay,
      double widthPixels1) {

    addCommands(
        new ParallelCommandGroup(
            new PreloadCargoCommand(magazineSubsystem),
            new OffsetGyroCommand(driveSubsystem, gyroOffset)),
        new ParallelCommandGroup(
            new WaitCommand(delay), new IgnoreColorSensorCommand(magazineSubsystem, true)),
        new ParallelDeadlineGroup(
            new DriveAutonCommand(driveSubsystem, path1Name, true, false), // deadline
            new ArmShooterCommandGroup(
                visionSubsystem, turretSubsystem, shooterSubsystem, driveSubsystem),
            new AutoIntakeCommand(
                magazineSubsystem, intakeSubsystem, intakeExtendSubsystem, true, true)),
        new VisionShootAutoCommand(
            shooterSubsystem,
            turretSubsystem,
            magazineSubsystem,
            visionSubsystem,
            false,
            intakeSubsystem,
            widthPixels1),
        new ParallelDeadlineGroup(
            new DriveAutonCommand(driveSubsystem, path2Name, false, true),
            new AutoIntakeCommand(
                magazineSubsystem, intakeSubsystem, intakeExtendSubsystem, true, true)),
        new WaitCommand(0.5),
        new OppCargoShotAutonCommand(
            turretSubsystem, shooterSubsystem, magazineSubsystem, intakeSubsystem));
    // Shoot opponent Ball
  }
}
