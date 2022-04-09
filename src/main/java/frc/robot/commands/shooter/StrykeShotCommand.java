package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class StrykeShotCommand extends CommandBase {

  public final TurretSubsystem turretSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  public final MagazineSubsystem magazineSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  private final Logger logger = LoggerFactory.getLogger(StrykeShotCommand.class);

  public StrykeShotCommand(
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.rotateTo(
        shooterSubsystem.isLeft
            ? ShooterConstants.kLeftTurretPos
            : ShooterConstants.kRightTurretPos);
    shooterSubsystem.strykeShot();
    magazineSubsystem.timedShoot();
  }

  @Override
  public boolean isFinished() {
    return magazineSubsystem.isShootSequenceDone();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    magazineSubsystem.magazineInterrupted();
    intakeSubsystem.openLoopRotate(0.0);
  }
}
