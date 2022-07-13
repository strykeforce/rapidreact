package frc.robot.commands.sequences.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class GeyserShootCommand extends CommandBase {
  public final TurretSubsystem turretSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  public final MagazineSubsystem magazineSubsystem;
  public final IntakeSubsystem intakeSubsystem;

  public GeyserShootCommand(
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    addRequirements(turretSubsystem, intakeSubsystem);
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.geyserShot(true);
    shooterSubsystem.geyserShot(false, true);
    magazineSubsystem.shoot();
  }

  @Override
  public boolean isFinished() {
    return magazineSubsystem.isShootSequenceDone();
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopTrackingTarget();
    shooterSubsystem.stop();
    magazineSubsystem.magazineInterrupted();
    intakeSubsystem.openLoopRotate(0.0);
  }
}
