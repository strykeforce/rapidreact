package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.OffsetGyroCommand;
import frc.robot.commands.magazine.IgnoreColorSensorCommand;
import frc.robot.commands.magazine.PreloadCargoCommand;
import frc.robot.commands.sequences.intaking.AutoIntakeCommand;
import frc.robot.commands.sequences.shooting.ArmShooterCommandGroup;
import frc.robot.commands.sequences.shooting.GeyserShootCommand;
import frc.robot.commands.sequences.shooting.VisionShootAutoCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DefenseAuto extends SequentialCommandGroup {

  public DefenseAuto(
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
            new ArmShooterCommandGroup(visionSubsystem, turretSubsystem, shooterSubsystem),
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
        new IgnoreColorSensorCommand(magazineSubsystem, true),
        new ParallelDeadlineGroup(
            new DriveAutonCommand(driveSubsystem, path2Name, false, true),
            new AutoIntakeCommand(
                magazineSubsystem, intakeSubsystem, intakeExtendSubsystem, true, true)),
        new WaitCommand(AutoConstants.kDefenseBallPickupDelay),
        new ParallelDeadlineGroup(
            new DriveAutonCommand(driveSubsystem, path3Name, false, true),
            new AutoIntakeCommand(
                magazineSubsystem, intakeSubsystem, intakeExtendSubsystem, true, true)),
        new IgnoreColorSensorCommand(magazineSubsystem, false),
        new WaitCommand(AutoConstants.kDefenseBallPickupDelay),
        new DriveAutonCommand(driveSubsystem, path4Name, false, true),
        new WaitMatchTimeCommand(AutoConstants.kWaitUntilMatchTime),
        new GeyserShootCommand(
            turretSubsystem, shooterSubsystem, magazineSubsystem, intakeSubsystem));
  }
}
