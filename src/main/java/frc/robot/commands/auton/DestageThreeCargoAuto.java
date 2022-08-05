package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.OffsetGyroCommand;
import frc.robot.commands.magazine.AutoSlowEjectCargoCommand;
import frc.robot.commands.magazine.AutonIgnoreColorSensorCommand;
import frc.robot.commands.magazine.PreloadCargoCommand;
import frc.robot.commands.sequences.intaking.AutoIntakeCommand;
import frc.robot.commands.sequences.shooting.ArmShooterCommandGroup;
import frc.robot.commands.sequences.shooting.UnknownOrderShotAutonCommand;
import frc.robot.commands.sequences.shooting.VisionShootAutoCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DestageThreeCargoAuto extends SequentialCommandGroup {

  public DestageThreeCargoAuto(
      VisionSubsystem visionSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeExtendSubsystem intakeExtendSubsystem,
      DriveSubsystem driveSubsystem,
      String path1Name,
      String path2Name,
      String path3Name,
      String path4Name,
      String path5Name,
      Rotation2d gyroOffset,
      double delay,
      double widthPixels) {
    addCommands(
        new ParallelCommandGroup(
            new PreloadCargoCommand(magazineSubsystem),
            new OffsetGyroCommand(driveSubsystem, gyroOffset)),
        new WaitCommand(delay),
        new ParallelDeadlineGroup(
            new DriveAutonCommand(driveSubsystem, path1Name, true, true), // deadline
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
            widthPixels),
        new AutonIgnoreColorSensorCommand(magazineSubsystem, true),
        new ParallelDeadlineGroup(
            new DriveAutonCommand(driveSubsystem, path2Name, false, true),
            new AutoIntakeCommand(
                magazineSubsystem, intakeSubsystem, intakeExtendSubsystem, true, true)),
        new DriveAutonCommand(driveSubsystem, path3Name, false, true),
        new AutoSlowEjectCargoCommand(magazineSubsystem, intakeSubsystem, shooterSubsystem),
        new ParallelDeadlineGroup(
            new DriveAutonCommand(driveSubsystem, path4Name, false, true),
            new AutoIntakeCommand(
                magazineSubsystem, intakeSubsystem, intakeExtendSubsystem, true, true)),
        new DriveAutonCommand(driveSubsystem, path5Name, false, true),
        new UnknownOrderShotAutonCommand(
            turretSubsystem, shooterSubsystem, magazineSubsystem, intakeSubsystem));
  }
}
