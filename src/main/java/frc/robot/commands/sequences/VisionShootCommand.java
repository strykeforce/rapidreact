package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.MagazineState;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretState;
import frc.robot.subsystems.VisionSubsystem;

public class VisionShootCommand extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private TurretSubsystem turretSubsystem;
  private MagazineSubsystem magazineSubsystem;
  private VisionSubsystem visionSubsystem;
  private boolean isArmed = false;

  public VisionShootCommand(
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      MagazineSubsystem magazineSubsystem,
      VisionSubsystem visionSubsystem) {
    addRequirements(shooterSubsystem, magazineSubsystem, turretSubsystem, visionSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void initialize() {
    // Check if Shooter Already Armed and Turret locked on
    if (shooterSubsystem.getCurrentState() == ShooterState.ARMED
        && turretSubsystem.getState() == TurretState.TRACKING) {
      isArmed = true;
      shooterSubsystem.shoot();
      magazineSubsystem.shoot();
    } else {
      isArmed = false;
      visionSubsystem.enable();
      shooterSubsystem.arm();
      turretSubsystem.trackTarget();
      turretSubsystem.resetSeekCount();
    }
  }

  @Override
  public void execute() {
    if (!isArmed
        && shooterSubsystem.getCurrentState() == ShooterState.ARMED
        && turretSubsystem.getState() == TurretState.TRACKING) {
      shooterSubsystem.shoot();
      magazineSubsystem.shoot();
      isArmed = true;
    }
  }

  @Override
  public boolean isFinished() {
    return magazineSubsystem.getCurrMagazineState() == MagazineState.STOP;
  }

  @Override
  public void end(boolean interrupted) {
    visionSubsystem.disable();
    shooterSubsystem.stop();
    magazineSubsystem.stopMagazine();
    turretSubsystem.stopTrackingTarget();
  }
}
