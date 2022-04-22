package frc.robot.commands.turret;

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
    double desiredTicks = SmartDashboard.getNumber(DashboardConstants.kTurretSetpointRadians, 0.0);
    turretSubsystem.rotateTo(desiredTicks);
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.isTurretAtTarget();
  }
}
