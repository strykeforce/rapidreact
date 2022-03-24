package frc.robot.commands.sequences.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretState;
import frc.robot.subsystems.VisionSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class VisionShootAutoCommand extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private TurretSubsystem turretSubsystem;
  private MagazineSubsystem magazineSubsystem;
  private VisionSubsystem visionSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private boolean isArmed = false;
  private double widthPixels;
  private final boolean disableTrackingOnFinish;
  private final Logger logger = LoggerFactory.getLogger(VisionShootCommand.class);

  public VisionShootAutoCommand(
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      MagazineSubsystem magazineSubsystem,
      VisionSubsystem visionSubsystem,
      boolean disableTrackingOnFinish,
      IntakeSubsystem intakeSubsystem,
      double widthPixels) {
    addRequirements(
        shooterSubsystem, magazineSubsystem, turretSubsystem, visionSubsystem, intakeSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.disableTrackingOnFinish = disableTrackingOnFinish;
    this.intakeSubsystem = intakeSubsystem;
    this.widthPixels = widthPixels;
  }

  @Override
  public void initialize() {
    // Check if Shooter Already Armed and Turret locked on
    logger.info("Shooting...");
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
      magazineSubsystem.shoot();
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
    } else if (!isArmed && turretSubsystem.getState() == TurretState.IDLE) {
      logger.info("Seek failed falling back to manual shoot, Width: {}", widthPixels);
      turretSubsystem.odometryAim();
      shooterSubsystem.manualShoot(widthPixels);
      magazineSubsystem.shoot();
    }
  }

  @Override
  public boolean isFinished() {
    return magazineSubsystem.isShootSequenceDone();
  }

  @Override
  public void end(boolean interrupted) {
    if (disableTrackingOnFinish) {
      visionSubsystem.disable();
      turretSubsystem.stopTrackingTarget();
    }
    shooterSubsystem.stop();
    magazineSubsystem.stopMagazine();
    intakeSubsystem.openLoopRotate(0.0);
  }
}
