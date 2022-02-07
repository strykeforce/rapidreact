package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HubTargetData;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TurretAimCommand extends CommandBase {

  private final VisionSubsystem visionSubsystem;
  private final TurretSubsystem turretSubsystem;
  public static final Logger logger = LoggerFactory.getLogger(TurretAimCommand.class);

  public TurretAimCommand(VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem, visionSubsystem);
    this.turretSubsystem = turretSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void initialize() {
    HubTargetData targetData = visionSubsystem.getTargetData();
    if (!targetData.isValid()) {
      logger.warn("target data invalid: {}", targetData);
      cancel();
      return;
    }

    Rotation2d rotationError = targetData.getErrorRotation2d();
    turretSubsystem.rotateTurret(rotationError);
    logger.info("rotating turret by {} deg.", rotationError.getDegrees());
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.isRotationFinished();
  }

  @Override
  public void end(boolean interrupted) {
    logger.info("rotation finished");
    visionSubsystem.disable();
  }
}
