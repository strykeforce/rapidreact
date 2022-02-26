package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TurretAimCommand extends CommandBase {

  public static final Logger logger = LoggerFactory.getLogger(TurretAimCommand.class);
  private final TurretSubsystem turretSubsystem;

  public TurretAimCommand(TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.resetSeekCount();
    turretSubsystem.trackTarget();
  }

  @Override
  public boolean isFinished() {
    // return turretSubsystem.getState() == TurretState.TRACKING;
    // return turretSubsystem.isRotationFinished();
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    logger.info("rotation finished");
    turretSubsystem.stopTrackingTarget();
  }
}
