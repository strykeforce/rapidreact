package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.intake.IntakeOpenLoopCommand;
import frc.robot.commands.magazine.UpperAndLowerMagazineClosedLoopTuningCommand;
import frc.robot.commands.shooter.PitHoodAndShooterClosedLoopCommand;
import frc.robot.commands.turret.RotateToCommand;
import frc.robot.commands.vision.EnableVisionCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PitShooterTuneCommandGroup extends ParallelCommandGroup {
  public PitShooterTuneCommandGroup(
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      TurretSubsystem turretSubsystem,
      VisionSubsystem visionSubsystem) {
    addCommands(
        new PitHoodAndShooterClosedLoopCommand(shooterSubsystem),
        new IntakeOpenLoopCommand(intakeSubsystem, IntakeConstants.kIntakeSpeed),
        new UpperAndLowerMagazineClosedLoopTuningCommand(magazineSubsystem),
        new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(0.0)),
        new EnableVisionCommand(visionSubsystem));
  }
}
