package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TurretAimCommand extends CommandBase {
  private final VisionSubsystem visionSubsystem;
  private final TurretSubsystem turretSubsystem;
  public final Logger logger = LoggerFactory.getLogger("Aim Shooter Command");

  public TurretAimCommand(VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem, visionSubsystem);
    this.turretSubsystem = turretSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void initialize() {
    Rotation2d rotationError = visionSubsystem.getErrorRotation2d();
    if (rotationError == null) return;

    // double offset = VISION.getOffsetAngle();
    // TURRET.rotateTurret(VISION.getAzmithError() /*offset + VISION.getHorizAngleAdjustment()*/);
    turretSubsystem.rotateTurret(rotationError);
    logger.info("TurretAimCommand (deg): {}", rotationError.getDegrees());
  }

  @Override
  public boolean isFinished() {
    // SmartDashboard.putBoolean("Match/Locked On", true);
    // return TURRET.turretAtTarget() && VISION.isStable() && VISION.isTargetValid();
    return turretSubsystem.isRotationFinished();
  }

  @Override
  public void end(boolean interrupted) {
    logger.info("TurretAtTarget: {}", turretSubsystem.isRotationFinished());
    visionSubsystem.shooterCamera.setEnabled(false);
  }
}
