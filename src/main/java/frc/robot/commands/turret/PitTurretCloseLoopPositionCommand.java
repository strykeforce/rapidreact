package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class PitTurretCloseLoopPositionCommand extends CommandBase {
  public TurretSubsystem turretSubsystem;

  public PitTurretCloseLoopPositionCommand(TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.setTurret(
        SmartDashboard.getNumber(
            "Pit/Turret/SetPointTicks", Constants.TurretConstants.kTurretMidpoint));
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.turretAtTarget();
  }
}
