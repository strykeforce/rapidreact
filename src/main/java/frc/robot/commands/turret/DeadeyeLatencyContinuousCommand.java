package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DeadeyeLatencyContinuousCommand extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final VisionSubsystem visionSubsystem;
  private boolean isPos;

  public DeadeyeLatencyContinuousCommand(
      TurretSubsystem turretSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(turretSubsystem, visionSubsystem);
    this.turretSubsystem = turretSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void initialize() {
    visionSubsystem.enable();
    turretSubsystem.rotateTo(Rotation2d.fromDegrees(90.0));
    isPos = true;
  }

  @Override
  public void execute() {
    if (turretSubsystem.isTurretAtTarget() && isPos) {
      isPos = false;
      turretSubsystem.rotateTo(Rotation2d.fromDegrees(-90.0));
    } else if (turretSubsystem.isTurretAtTarget() && !isPos) {
      isPos = true;
      turretSubsystem.rotateTo(Rotation2d.fromDegrees(90.0));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
