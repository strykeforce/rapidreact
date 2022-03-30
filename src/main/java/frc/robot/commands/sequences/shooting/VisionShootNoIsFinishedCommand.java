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

public class VisionShootNoIsFinishedCommand extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private TurretSubsystem turretSubsystem;
  private MagazineSubsystem magazineSubsystem;
  private VisionSubsystem visionSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private boolean isArmed = false;
  private double widthPixels;
  private final boolean disableTrackingOnFinish;
  private final Logger logger = LoggerFactory.getLogger(VisionShootNoIsFinishedCommand.class);

  public VisionShootNoIsFinishedCommand(
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      MagazineSubsystem magazineSubsystem,
      VisionSubsystem visionSubsystem,
      boolean disableTrackingOnFinish,
      IntakeSubsystem intakeSubsystem,
      double widthPixels) {
    addRequirements(shooterSubsystem, turretSubsystem, magazineSubsystem, visionSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.disableTrackingOnFinish = disableTrackingOnFinish;
    this.widthPixels = widthPixels;
  }

  @Override
  public void initialize() {
    logger.info("Shoot No Is Finished");
    if (turretSubsystem.getState() == TurretState.TRACKING) {
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
    } else if (!isArmed && turretSubsystem.getState() == TurretState.IDLE) {
      logger.info("Seek Failed: falling back to manual, width: {}", widthPixels);
      turretSubsystem.odometryAim();
      shooterSubsystem.manualShoot(widthPixels);
      magazineSubsystem.shoot();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
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
