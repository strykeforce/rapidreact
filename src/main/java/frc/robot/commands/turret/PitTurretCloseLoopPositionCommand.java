package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.subsystems.TurretSubsystem;

public class PitTurretCloseLoopPositionCommand extends CommandBase {
  public final TurretSubsystem turretSubsystem;

  public PitTurretCloseLoopPositionCommand(TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    Rotation2d desired =
        new Rotation2d(
            SmartDashboard.getNumber(
                DashboardConstants.kTurretSetpointRadians,
                turretSubsystem.getRotation2d().getRadians()));
    turretSubsystem.rotateBy(desired);
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.isRotationFinished();
  }
}
