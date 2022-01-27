package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TurretAimCommand extends InstantCommand {
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
      // VISION.ShooterCamera.setEnabled(true);
      // double offset = VISION.getOffsetAngle();
      // TURRET.setTurret(VISION.getAzmithError());
      // TURRET.rotateTurret(VISION.getAzmithError() /*offset + VISION.getHorizAngleAdjustment()*/);
      System.out.println("TurretAimCommand: " + visionSubsystem.getAzmithError());
      // VISION.getSortedTargets();
    }
  }

  /*@Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Match/Locked On", true);
    return TURRET.turretAtTarget() && VISION.isStable() && VISION.isTargetValid();
  }*/
}
