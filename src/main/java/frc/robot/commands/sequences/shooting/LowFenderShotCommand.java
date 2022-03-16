package frc.robot.commands.sequences.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class LowFenderShotCommand extends CommandBase {
  public final TurretSubsystem turretSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  public final MagazineSubsystem magazineSubsystem;

  public LowFenderShotCommand(
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem) {
    addRequirements(turretSubsystem, shooterSubsystem, magazineSubsystem);
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.magazineSubsystem = magazineSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.fenderShot(true);
    shooterSubsystem.fenderShot(false);
    magazineSubsystem.shoot();
  }

  @Override
  public boolean isFinished() {
    return magazineSubsystem.isMagazineEmpty() && !magazineSubsystem.isUpperBeamBroken();
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopTrackingTarget();
    shooterSubsystem.stop();
    if (interrupted) {
      magazineSubsystem.magazineInterrupted();
    }
  }
}
