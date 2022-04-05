package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.OffsetGyroCommand;
import frc.robot.commands.magazine.PreloadCargoCommand;
import frc.robot.commands.sequences.intaking.AutoIntakeCommand;
import frc.robot.commands.sequences.shooting.ArmShooterCommandGroup;
import frc.robot.commands.sequences.shooting.VisionShootAutoCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ThreeCargoAuto extends SequentialCommandGroup {

  public ThreeCargoAuto(
      VisionSubsystem visionSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      DriveSubsystem driveSubsystem,
      String path1Name,
      String path2Name,
      Rotation2d gyroOffset,
      double delay,
      double widthPixels1,
      double widthPixels2) {
    addCommands(
        new ParallelCommandGroup(
            new PreloadCargoCommand(magazineSubsystem),
            new OffsetGyroCommand(driveSubsystem, gyroOffset)),
        new WaitCommand(delay),
        new ParallelDeadlineGroup(
            new DriveAutonCommand(driveSubsystem, path1Name, true, false), // deadline
            new ArmShooterCommandGroup(visionSubsystem, turretSubsystem, shooterSubsystem),
            new AutoIntakeCommand(magazineSubsystem, intakeSubsystem, true, true)),
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
            new ArmShooterCommandGroup(visionSubsystem, turretSubsystem, shooterSubsystem),
            new AutoIntakeCommand(magazineSubsystem, intakeSubsystem, true, true)),
        new VisionShootAutoCommand(
            shooterSubsystem,
            turretSubsystem,
            magazineSubsystem,
            visionSubsystem,
            true,
            intakeSubsystem,
            widthPixels2));
  }
}
