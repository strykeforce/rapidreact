package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TurretAimCommand extends CommandBase {
  private VisionSubsystem visionSubsystem;
  private TurretSubsystem turretSubsystem;
  public Logger logger = LoggerFactory.getLogger("Aim Shooter Command");

  public TurretAimCommand(VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem, visionSubsystem);
    this.turretSubsystem = turretSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void initialize() {
    if (visionSubsystem.isTargetValid()) {
      // double offset = VISION.getOffsetAngle();
      // TURRET.rotateTurret(VISION.getAzmithError() /*offset + VISION.getHorizAngleAdjustment()*/);
      turretSubsystem.rotateTurret(visionSubsystem.getErrorRotation2d());
      System.out.println(
          "TurretAimCommand (deg): " + visionSubsystem.getErrorRotation2d().getDegrees());
    }
  }

  @Override
  public boolean isFinished() {
    // SmartDashboard.putBoolean("Match/Locked On", true);
    // return TURRET.turretAtTarget() && VISION.isStable() && VISION.isTargetValid();
    return turretSubsystem.turretAtTarget();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("TurretAtTarget: " + turretSubsystem.turretAtTarget());
    visionSubsystem.shooterCamera.setEnabled(false);
  }
}
