package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SmartDashboardConstants;
import frc.robot.subsystems.TurretSubsystem;

public class PitTurretCloseLoopPositionCommand extends CommandBase {
  public TurretSubsystem turretSubsystem;

  public PitTurretCloseLoopPositionCommand(TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    Rotation2d desired =
        new Rotation2d(
            SmartDashboard.getNumber(
                SmartDashboardConstants.kTurretSetpointRadians,
                turretSubsystem.getRotation2d().getRadians()));
    turretSubsystem.rotateTo(desired);
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.turretAtTarget();
  }
}
