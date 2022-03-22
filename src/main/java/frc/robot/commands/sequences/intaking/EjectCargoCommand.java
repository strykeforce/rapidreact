package frc.robot.commands.sequences.intaking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class EjectCargoCommand extends CommandBase {
  public final TurretSubsystem turretSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  public final MagazineSubsystem magazineSubsystem;
  public final IntakeSubsystem intakeSubsystem;

  public EjectCargoCommand(
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    addRequirements(turretSubsystem, shooterSubsystem, magazineSubsystem, intakeSubsystem);
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.fenderShot(false);
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
    intakeSubsystem.openLoopRotate(0.0);
  }
}
