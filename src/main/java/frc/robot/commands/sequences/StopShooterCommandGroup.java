package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.IntakeOpenLoopCommand;
import frc.robot.commands.magazine.StopMagazineCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.commands.turret.StopTurretCommand;
import frc.robot.commands.vision.DisableVisionCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class StopShooterCommandGroup extends ParallelCommandGroup {
  public StopShooterCommandGroup(
      MagazineSubsystem magazineSubsystem,
      VisionSubsystem visionSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem) {
    addCommands(
        new StopMagazineCommand(magazineSubsystem),
        new DisableVisionCommand(visionSubsystem),
        new StopTurretCommand(turretSubsystem),
        new StopShooterCommand(shooterSubsystem),
        new IntakeOpenLoopCommand(intakeSubsystem, 0.0));
  }
}
